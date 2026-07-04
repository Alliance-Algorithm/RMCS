#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <format>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include "predefined_msg_provider.hpp"
#include "rmcs_executor/component.hpp"
#include "rmcs_utility/tdigest.hpp"
#include "rmcs_utility/thread_config.hpp"

namespace rmcs_executor {

class Executor final : public rclcpp::Node {
public:
    explicit Executor(
        const std::string& node_name, rclcpp::executors::SingleThreadedExecutor& rcl_executor)
        : Node{
              node_name,
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)}
        , rcl_executor_(rcl_executor) {
        Component::initializing_component_name = "predefined_msg_provider";
        predefined_msg_provider_ = std::make_shared<PredefinedMsgProvider>();
        add_component(predefined_msg_provider_);
    }
    ~Executor() override {
        disable_event_outputs();
        wait_event_outputs_idle();
        if (thread_.joinable())
            thread_.join();
    };

    void add_component(const std::shared_ptr<Component>& component) {
        component_list_.emplace_back(component);
        if (auto node = std::dynamic_pointer_cast<rclcpp::Node>(component))
            rcl_executor_.add_node(node);
        for (auto& partner_component : component->partner_component_list_)
            add_component(partner_component);
    }

    void start() {
        init();

        for (auto& component : component_list_)
            component->before_updating();

        double update_rate;
        if (!get_parameter("update_rate", update_rate))
            throw std::runtime_error{"Unable to get parameter update_rate<double>"};
        predefined_msg_provider_->set_update_rate(update_rate);

        std::string thread_config_spec;
        get_parameter_or<std::string>("thread_config", thread_config_spec, "");

        enable_event_outputs();

        thread_ = std::thread{
            [this, update_rate,
             thread_config = rmcs_utility::ThreadConfig{thread_config_spec, "update"}]() {
                thread_main(update_rate, thread_config);
            }};
    }

private:
    using SteadyClock = std::chrono::steady_clock;

    static constexpr size_t digest_size_ = 256;

    struct CumulativeStats {
        rmcs_utility::TDigest<double> start_lateness_ms{digest_size_};
        rmcs_utility::TDigest<double> update_duration_ms{digest_size_};
        uint64_t update_count = 0;
        uint64_t skipped_count = 0;
        uint64_t start_lateness_sample_count = 0;
        uint64_t update_duration_sample_count = 0;
        double start_lateness_max_ms = 0.0;
        double update_duration_max_ms = 0.0;
    };

    struct WindowStats {
        explicit WindowStats(size_t component_count)
            : component_update_duration_sums(component_count) {}

        uint64_t executed_count = 0;
        SteadyClock::duration total_update_duration{};
        std::vector<SteadyClock::duration> component_update_duration_sums;
    };

    struct ThreadStats {
        explicit ThreadStats(size_t component_count, SteadyClock::time_point start_time)
            : window(component_count)
            , next_report_time(start_time + std::chrono::seconds(5)) {}

        CumulativeStats cumulative;
        WindowStats window;
        SteadyClock::time_point next_report_time;
    };

    struct ComponentWindowStat {
        size_t index;
        SteadyClock::duration duration_sum;
    };

    void thread_main(double update_rate, const rmcs_utility::ThreadConfig& thread_config) {
        if (const auto result = thread_config.apply_to_current_thread(); !result)
            RCLCPP_WARN(get_logger(), "%s", result.error().c_str());

        const auto period =
            std::chrono::nanoseconds(static_cast<long>(std::round(1'000'000'000.0 / update_rate)));
        auto next_iteration_time = SteadyClock::now();
        auto stats = ThreadStats{updating_order_.size(), next_iteration_time};

        try {
            while (rclcpp::ok()) {
                const auto scheduled_start = next_iteration_time;
                const auto actual_end = execute_update_iteration(scheduled_start, stats);
                next_iteration_time = calculate_next_iteration_time(
                    scheduled_start, actual_end, period, stats.cumulative);
                log_due_reports(actual_end, stats);
                std::this_thread::sleep_until(next_iteration_time);
            }
        } catch (const std::exception& exception) {
            RCLCPP_FATAL(
                get_logger(), "Executor update thread terminated by exception: %s",
                exception.what());
            rclcpp::shutdown();
        } catch (...) {
            RCLCPP_FATAL(get_logger(), "Executor update thread terminated by unknown exception");
            rclcpp::shutdown();
        }
    }

    SteadyClock::time_point
        execute_update_iteration(SteadyClock::time_point scheduled_start, ThreadStats& stats) {
        auto current_time = SteadyClock::now();
        const auto actual_start = current_time;

        record_start_lateness(scheduled_start, actual_start, stats.cumulative);

        stats.cumulative.update_count++;
        stats.window.executed_count++;

        predefined_msg_provider_->set_timestamp(scheduled_start);

        for (size_t component_index = 0; component_index < updating_order_.size();
             ++component_index) {
            const auto component_start_time = current_time;
            updating_order_[component_index]->update();
            current_time = SteadyClock::now();
            stats.window.component_update_duration_sums[component_index] +=
                current_time - component_start_time;
        }

        const auto update_duration = current_time - actual_start;
        record_update_duration(update_duration, stats);
        return current_time;
    }

    void record_start_lateness(
        SteadyClock::time_point scheduled_start, SteadyClock::time_point actual_start,
        CumulativeStats& stats) {
        if (actual_start < scheduled_start) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000, "Executor update thread woke early by %.3f ms",
                duration_to_ms(scheduled_start - actual_start));
            return;
        }

        const auto start_lateness_ms = duration_to_ms(actual_start - scheduled_start);
        stats.start_lateness_ms.insert(start_lateness_ms);
        stats.start_lateness_max_ms = std::max(stats.start_lateness_max_ms, start_lateness_ms);
        stats.start_lateness_sample_count++;
    }

    static void record_update_duration(SteadyClock::duration update_duration, ThreadStats& stats) {
        const auto update_duration_ms = duration_to_ms(update_duration);
        stats.cumulative.update_duration_ms.insert(update_duration_ms);
        stats.cumulative.update_duration_max_ms =
            std::max(stats.cumulative.update_duration_max_ms, update_duration_ms);
        stats.cumulative.update_duration_sample_count++;
        stats.window.total_update_duration += update_duration;
    }

    static SteadyClock::time_point calculate_next_iteration_time(
        SteadyClock::time_point scheduled_start, SteadyClock::time_point actual_end,
        std::chrono::nanoseconds period, CumulativeStats& stats) {
        auto next_iteration_time = scheduled_start + period;
        if (actual_end <= next_iteration_time)
            return next_iteration_time;

        const auto overdue_duration = actual_end - next_iteration_time;
        const auto skipped_cycles =
            static_cast<uint64_t>(
                (overdue_duration - std::chrono::nanoseconds{1}).count() / period.count())
            + 1;
        stats.skipped_count += skipped_cycles;
        stats.update_count += skipped_cycles;
        return next_iteration_time + period * skipped_cycles;
    }

    void log_due_reports(SteadyClock::time_point current_time, ThreadStats& stats) {
        while (current_time >= stats.next_report_time) {
            log_cumulative_stats(stats.cumulative);
            log_window_stats(stats.window);
            reset_window_stats(stats.window);
            stats.next_report_time += std::chrono::seconds(5);
        }
    }

    void log_cumulative_stats(const CumulativeStats& stats) {
        const double skipped_ratio = stats.update_count == 0
                                       ? 0.0
                                       : static_cast<double>(stats.skipped_count)
                                             / static_cast<double>(stats.update_count) * 100.0;

        RCLCPP_INFO(
            get_logger(),
            "Update/Skipped: %llu/%llu (%s%%), "
            "Stat ms p50/p99/max: start_late %s/%s/%s, update %s/%s/%s",
            static_cast<unsigned long long>(stats.update_count),
            static_cast<unsigned long long>(stats.skipped_count),
            format_percent(skipped_ratio).c_str(),
            format_stat(
                maybe_quantile(stats.start_lateness_ms, stats.start_lateness_sample_count, 50.0))
                .c_str(),
            format_stat(
                maybe_quantile(stats.start_lateness_ms, stats.start_lateness_sample_count, 99.0))
                .c_str(),
            format_stat(
                stats.start_lateness_sample_count == 0
                    ? std::nullopt
                    : std::optional<double>{stats.start_lateness_max_ms})
                .c_str(),
            format_stat(
                maybe_quantile(stats.update_duration_ms, stats.update_duration_sample_count, 50.0))
                .c_str(),
            format_stat(
                maybe_quantile(stats.update_duration_ms, stats.update_duration_sample_count, 99.0))
                .c_str(),
            format_stat(
                stats.update_duration_sample_count == 0
                    ? std::nullopt
                    : std::optional<double>{stats.update_duration_max_ms})
                .c_str());
    }

    void log_window_stats(const WindowStats& stats) {
        if (stats.executed_count == 0
            || stats.total_update_duration == SteadyClock::duration::zero())
            return;

        auto top_component_stats = collect_top_component_stats(stats);
        if (top_component_stats.empty())
            return;

        RCLCPP_INFO(
            get_logger(), "Component 5s: %s",
            format_top_component_stats(top_component_stats, stats).c_str());
    }

    static std::vector<ComponentWindowStat> collect_top_component_stats(const WindowStats& stats) {
        auto top_component_stats = std::vector<ComponentWindowStat>{};
        top_component_stats.reserve(stats.component_update_duration_sums.size());
        for (size_t component_index = 0;
             component_index < stats.component_update_duration_sums.size(); ++component_index) {
            const auto duration_sum = stats.component_update_duration_sums[component_index];
            if (duration_sum == SteadyClock::duration::zero())
                continue;
            top_component_stats.emplace_back(component_index, duration_sum);
        }

        const size_t top_count = std::min<size_t>(3, top_component_stats.size());
        std::partial_sort(
            top_component_stats.begin(),
            top_component_stats.begin() + static_cast<std::ptrdiff_t>(top_count),
            top_component_stats.end(), [](const auto& lhs, const auto& rhs) {
                if (lhs.duration_sum != rhs.duration_sum)
                    return lhs.duration_sum > rhs.duration_sum;
                return lhs.index < rhs.index;
            });
        top_component_stats.resize(top_count);
        return top_component_stats;
    }

    std::string format_top_component_stats(
        const std::vector<ComponentWindowStat>& top_component_stats,
        const WindowStats& stats) const {
        auto top_components = std::string{};
        for (size_t rank = 0; rank < top_component_stats.size(); ++rank) {
            if (rank != 0)
                top_components += ", ";

            const auto& stat = top_component_stats[rank];
            top_components += std::format(
                "{} {}ms/{}%", updating_order_[stat.index]->get_component_name(),
                format_stat(
                    std::optional<double>{
                        duration_to_ms(stat.duration_sum)
                        / static_cast<double>(stats.executed_count)}),
                format_percent(
                    duration_to_ms(stat.duration_sum) / duration_to_ms(stats.total_update_duration)
                    * 100.0));
        }
        return top_components;
    }

    static void reset_window_stats(WindowStats& stats) {
        stats.executed_count = 0;
        stats.total_update_duration = SteadyClock::duration::zero();
        std::fill(
            stats.component_update_duration_sums.begin(),
            stats.component_update_duration_sums.end(), SteadyClock::duration::zero());
    }

    static double duration_to_ms(SteadyClock::duration duration) {
        return std::chrono::duration<double, std::milli>(duration).count();
    }

    static std::string format_stat(const std::optional<double>& value) {
        if (!value.has_value())
            return "n/a";
        return std::format("{:.3f}", *value);
    }

    static std::string format_percent(double value) { return std::format("{:.1f}", value); }

    static std::optional<double> maybe_quantile(
        const rmcs_utility::TDigest<double>& digest, uint64_t sample_count, double quantile) {
        if (sample_count == 0)
            return std::nullopt;
        return digest.quantile(quantile);
    }

    void enable_event_outputs() {
        for (const auto& component : component_list_) {
            for (const auto& output : component->output_list_) {
                if (output.enable == nullptr)
                    continue;
                output.enable(output.binding);
            }
        }
    }

    void disable_event_outputs() {
        for (const auto& component : component_list_) {
            for (const auto& output : component->output_list_) {
                if (output.disable == nullptr)
                    continue;
                output.disable(output.binding);
            }
        }
    }

    void wait_event_outputs_idle() {
        for (const auto& component : component_list_) {
            for (const auto& output : component->output_list_) {
                if (output.wait_idle == nullptr)
                    continue;
                output.wait_idle(output.binding);
            }
        }
    }

    struct InterfaceKey {
        std::string name;
        InterfaceKind kind;

        bool operator==(const InterfaceKey& other) const {
            return name == other.name && kind == other.kind;
        }
    };

    struct InterfaceKeyHash {
        std::size_t operator()(const InterfaceKey& key) const {
            return std::hash<std::string>{}(key.name)
                 ^ (std::hash<int>{}(static_cast<int>(key.kind)) << 1U);
        }
    };

    using OutputRecord = Component::OutputDeclaration*;

    struct NameKindRecord {
        InterfaceKind kind;
        std::string component_name;
        const char* direction;
    };

    static std::string describe_output(
        const std::string& name, InterfaceKind kind, const std::type_info& type,
        const std::string& component_name) {
        std::ostringstream stream;
        stream << "Component [" << component_name << "] registered " << interface_kind_name(kind)
               << " output \"" << name << "\" with type \"" << type.name() << "\"";
        return stream.str();
    }

    static std::string describe_output(const Component::OutputDeclaration& output) {
        return describe_output(
            output.name, output.kind, output.type, output.component->get_component_name());
    }

    static std::string describe_declaration(
        const std::string& name, InterfaceKind kind, const std::string& component_name,
        const char* direction) {
        std::ostringstream stream;
        stream << "Component [" << component_name << "] registered " << interface_kind_name(kind)
               << ' ' << direction << " \"" << name << "\"";
        return stream.str();
    }

    void init() {
        updating_order_.clear();

        auto output_map = std::unordered_map<InterfaceKey, OutputRecord, InterfaceKeyHash>{};
        auto output_by_name = std::unordered_map<std::string, OutputRecord>{};
        auto declaration_kind_map = std::unordered_map<std::string, NameKindRecord>{};
        auto user_output_map = Component::OutputInfoMap{};
        for (const auto& component : component_list_) {
            component->dependency_count_ = 0;
            component->wanted_by_.clear();
            for (auto& output : component->output_list_) {
                auto declaration_iter = declaration_kind_map.find(output.name);
                if (declaration_iter != declaration_kind_map.end()
                    && declaration_iter->second.kind != output.kind) {
                    const auto message =
                        std::string{"Conflicting interface kinds for name \""} + output.name
                        + "\": "
                        + describe_declaration(
                            output.name, declaration_iter->second.kind,
                            declaration_iter->second.component_name,
                            declaration_iter->second.direction)
                        + "; "
                        + describe_declaration(
                            output.name, output.kind, component->get_component_name(), "output")
                        + ". A name can only belong to one interface kind.";
                    RCLCPP_FATAL(get_logger(), "%s", message.c_str());
                    throw std::runtime_error{message};
                }
                declaration_kind_map.emplace(
                    output.name,
                    NameKindRecord{output.kind, component->get_component_name(), "output"});

                auto existing_output_iter = output_by_name.find(output.name);
                if (existing_output_iter != output_by_name.end()
                    && existing_output_iter->second->kind != output.kind) {
                    const auto& existing_output = *existing_output_iter->second;
                    const auto message = std::string{"Conflicting output kinds for name \""}
                                       + output.name + "\": " + describe_output(existing_output)
                                       + "; " + describe_output(output)
                                       + ". A name can only belong to one interface kind.";
                    RCLCPP_FATAL(get_logger(), "%s", message.c_str());
                    throw std::runtime_error{message};
                }

                output_by_name.emplace(output.name, &output);

                const auto [output_iter, inserted] =
                    output_map.emplace(InterfaceKey{output.name, output.kind}, &output);
                if (!inserted) {
                    const auto& existing_output = *output_iter->second;
                    const auto message =
                        std::string{"Duplicate "} + interface_kind_name(output.kind)
                        + " output name \"" + output.name
                        + "\": " + describe_output(existing_output) + "; " + describe_output(output)
                        + ". Only one output may be registered for each (name, kind).";
                    RCLCPP_FATAL(get_logger(), "%s", message.c_str());
                    throw std::runtime_error{message};
                }

                user_output_map.emplace(
                    output.name, Component::OutputInfo{std::cref(output.type), output.kind});
            }

            for (const auto& input : component->input_list_) {
                auto declaration_iter = declaration_kind_map.find(input.name);
                if (declaration_iter != declaration_kind_map.end()
                    && declaration_iter->second.kind != input.kind) {
                    const auto message =
                        std::string{"Conflicting interface kinds for name \""} + input.name + "\": "
                        + describe_declaration(
                            input.name, declaration_iter->second.kind,
                            declaration_iter->second.component_name,
                            declaration_iter->second.direction)
                        + "; "
                        + describe_declaration(
                            input.name, input.kind, component->get_component_name(), "input")
                        + ". A name can only belong to one interface kind.";
                    RCLCPP_FATAL(get_logger(), "%s", message.c_str());
                    throw std::runtime_error{message};
                }
                declaration_kind_map.emplace(
                    input.name,
                    NameKindRecord{input.kind, component->get_component_name(), "input"});
            }
        }

        for (auto& component : component_list_) {
            component->before_pairing(user_output_map);
        }

        for (const auto& component : component_list_) {
            for (const auto& input : component->input_list_) {
                auto output_iter = output_map.find(InterfaceKey{input.name, input.kind});
                if (output_iter == output_map.end()) {
                    auto output_by_name_iter = output_by_name.find(input.name);
                    if (output_by_name_iter != output_by_name.end()) {
                        const auto& available_output = *output_by_name_iter->second;
                        const auto message =
                            std::string{"Component ["} + component->get_component_name()
                            + "] requested " + interface_kind_name(input.kind) + " input \""
                            + input.name + "\", but " + describe_output(available_output) + ".";

                        RCLCPP_FATAL(get_logger(), "%s", message.c_str());
                        throw std::runtime_error{message};
                    }

                    if (!input.required)
                        continue;

                    const auto message = std::string{"Cannot find "}
                                       + interface_kind_name(input.kind) + " output \"" + input.name
                                       + "\" required by component ["
                                       + component->get_component_name() + "].";
                    RCLCPP_FATAL(get_logger(), "%s", message.c_str());
                    throw std::runtime_error{message};
                }

                const auto& output = *output_iter->second;
                if (input.type != output.type) {
                    const auto message = std::string{"Type mismatch for "}
                                       + interface_kind_name(input.kind) + " interface \""
                                       + input.name + "\": component ["
                                       + output.component->get_component_name()
                                       + "] declared output type \"" + output.type.name()
                                       + "\", but component [" + component->get_component_name()
                                       + "] requested input type \"" + input.type.name() + "\".";
                    RCLCPP_FATAL(get_logger(), "%s", message.c_str());
                    RCLCPP_FATAL(
                        get_logger(), "    Component [%s] declared the output with type \"%s\"",
                        output.component->get_component_name().c_str(), output.type.name());
                    RCLCPP_FATAL(
                        get_logger(), "    Component [%s] requested the input with type \"%s\"",
                        component->get_component_name().c_str(), input.type.name());
                    throw std::runtime_error{message};
                }

                if (output.component->wanted_by_.emplace(component.get()).second)
                    component->dependency_count_++;

                input.bind(input.binding, output.binding);
            }
        }

        RCLCPP_INFO(get_logger(), "Calculating component dependencies");
        std::vector<Component*> independent_list;
        for (const auto& component : component_list_) {
            if (component->dependency_count_ == 0)
                independent_list.emplace_back(component.get());
        }
        for (const auto& component : independent_list) {
            append_updating_order(component);
        }

        if (updating_order_.size() < component_list_.size()) {
            RCLCPP_FATAL(get_logger(), "Circular dependency found:");
            for (const auto& component : component_list_) {
                if (component->dependency_count_ == 0)
                    continue;

                RCLCPP_FATAL(
                    get_logger(), "Component [%s]:", component->get_component_name().c_str());
                for (const auto& input : component->input_list_) {
                    const auto output_iter = output_map.find(InterfaceKey{input.name, input.kind});
                    if (output_iter == output_map.end())
                        continue;

                    const auto* output = output_iter->second;
                    if (output->component->dependency_count_ == 0)
                        continue;
                    RCLCPP_FATAL(
                        get_logger(), "    Depends on [%s] because requesting %s interface \"%s\"",
                        output->component->component_name_.c_str(), interface_kind_name(input.kind),
                        output->name.c_str());
                }
            }
            throw std::runtime_error{"Circular dependency found"};
        }
    }

    void append_updating_order(Component* updatable_component) {
        std::string space = "- ";
        for (size_t i = dependency_recursive_level_; i-- > 0;)
            space.append("    ");
        RCLCPP_INFO(
            get_logger(), "%s%s", space.c_str(), updatable_component->get_component_name().c_str());
        updating_order_.emplace_back(updatable_component);

        for (auto& component : component_list_) {
            if (updatable_component->wanted_by_.contains(component.get())) {
                if (--component->dependency_count_ == 0) {
                    dependency_recursive_level_++;
                    append_updating_order(component.get());
                    dependency_recursive_level_--;
                }
            }
        }
    };

    rclcpp::executors::SingleThreadedExecutor& rcl_executor_;

    std::thread thread_;

    std::shared_ptr<PredefinedMsgProvider> predefined_msg_provider_;
    std::vector<std::shared_ptr<Component>> component_list_;

    std::vector<Component*> updating_order_;
    size_t dependency_recursive_level_ = 0;
};

} // namespace rmcs_executor
