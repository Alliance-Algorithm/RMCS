#pragma once

#include <expected>
#include <format>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <cerrno>
#include <charconv>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace rmcs_utility {

/**
 * Parsed thread configuration.
 *
 * Supported fields in `spec` are `name`, `cpus`, `policy`, `priority`, and `nice`.
 * Format: `key=value;key=value;...`
 * Empty specs are allowed and represent a no-op configuration.
 * Examples:
 * - `name=rmcs-ctrl;cpus=3;policy=fifo;priority=80`
 * - `cpus=4-7;policy=other;nice=-5`
 */
class ThreadConfig {
public:
    /**
     * Parse a thread configuration spec.
     *
     * @param spec Thread configuration spec string.
     * @throws std::invalid_argument If the spec is malformed or contains invalid values.
     */
    explicit ThreadConfig(std::string_view spec) {
        spec = trim(spec);
        parse_fields(spec);
        validate(spec);
    }

    /**
     * Parse a thread configuration spec and fill in a default thread name when `name` is unset.
     *
     * @param spec Thread configuration spec string.
     * @param default_name Fallback name used only when `spec` does not configure `name`.
     * @throws std::invalid_argument If the spec is malformed, contains invalid values, or
     * `default_name` is invalid when used.
     */
    ThreadConfig(std::string_view spec, std::string_view default_name)
        : ThreadConfig(spec) {
        if (name_)
            return;
        if (const auto invalid_reason = check_name(default_name))
            throw std::invalid_argument(std::string(*invalid_reason));
        name_ = to_string(default_name);
    }

    [[nodiscard]] auto name() const -> const std::optional<std::string>& { return name_; }
    [[nodiscard]] auto cpus() const -> const std::optional<cpu_set_t>& { return cpus_; }
    [[nodiscard]] auto policy() const -> const std::optional<int>& { return policy_; }
    [[nodiscard]] auto priority() const -> const std::optional<int>& { return priority_; }
    [[nodiscard]] auto nice() const -> const std::optional<int>& { return nice_; }

    /**
     * Apply the stored configuration to the calling thread.
     *
     * This operation is not atomic. If a later step fails, earlier changes may already have been
     * applied to the current thread.
     *
     * @return `std::expected<void, std::string>{}` on success, or an error string describing the
     * failing runtime step.
     */
    auto apply_to_current_thread() const -> std::expected<void, std::string> {
        const auto current_thread = pthread_self();

        if (policy_) {
            sched_param param{};
            param.sched_priority = priority_.value_or(0);
            const int error_code = pthread_setschedparam(current_thread, *policy_, &param);
            if (error_code != 0) {
                return std::unexpected(
                    std::format(
                        "Failed to set thread scheduling policy: {}", std::strerror(error_code)));
            }
        }

        if (nice_) {
            errno = 0;
            const auto tid = static_cast<id_t>(syscall(SYS_gettid));
            if (setpriority(PRIO_PROCESS, tid, *nice_) != 0)
                return std::unexpected(
                    std::format("Failed to set thread nice value: {}", std::strerror(errno)));
        }

        if (cpus_) {
            const int error_code =
                pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &*cpus_);
            if (error_code != 0)
                return std::unexpected(
                    std::format("Failed to set thread affinity: {}", std::strerror(error_code)));
        }

        if (name_) {
            const int error_code = pthread_setname_np(current_thread, name_->c_str());
            if (error_code != 0)
                return std::unexpected(
                    std::format("Failed to set thread name: {}", std::strerror(error_code)));
        }

        return {};
    }

private:
    static auto trim(std::string_view text) -> std::string_view {
        while (!text.empty() && (text.front() == ' ' || text.front() == '\t'))
            text.remove_prefix(1);
        while (!text.empty() && (text.back() == ' ' || text.back() == '\t'))
            text.remove_suffix(1);
        return text;
    }

    static auto to_string(std::string_view text) -> std::string { return std::string{text}; }

    [[noreturn]] static void throw_invalid_spec(std::string_view reason, std::string_view spec) {
        throw std::invalid_argument(
            std::format("Invalid thread config spec ({}): \"{}\"", reason, spec));
    }

    static auto parse_int(std::string_view value, std::string_view key, std::string_view spec)
        -> int {
        int result = 0;
        const auto* begin = value.data();
        const auto* end = value.data() + value.size();
        const auto [ptr, error_code] = std::from_chars(begin, end, result);
        if (error_code != std::errc{} || ptr != end) {
            throw_invalid_spec(std::format("invalid integer for key '{}'", key), spec);
        }
        return result;
    }

    static auto check_name(std::string_view name) -> std::optional<std::string> {
        if (name.empty())
            return "Thread name must not be empty";
        if (name.size() > 15)
            return std::format("Thread name exceeds 15 characters: \"{}\"", name);
        return std::nullopt;
    }

    void parse_fields(std::string_view spec) {
        bool seen_name = false;
        bool seen_cpus = false;
        bool seen_policy = false;
        bool seen_priority = false;
        bool seen_nice = false;

        auto remaining_spec = spec;
        while (true) {
            const auto separator = remaining_spec.find(';');
            const auto field = trim(remaining_spec.substr(0, separator));

            if (!field.empty()) {
                const auto equals = field.find('=');
                if (equals == std::string_view::npos)
                    throw_invalid_spec("missing '='", spec);

                const auto key = trim(field.substr(0, equals));
                const auto value = trim(field.substr(equals + 1));
                if (key.empty() || value.empty())
                    throw_invalid_spec("empty key or value", spec);

                assign_field(
                    key, value, spec, seen_name, seen_cpus, seen_policy, seen_priority, seen_nice);
            }

            if (separator == std::string_view::npos)
                break;
            remaining_spec.remove_prefix(separator + 1);
        }
    }

    void assign_field(
        std::string_view key, std::string_view value, std::string_view spec, bool& seen_name,
        bool& seen_cpus, bool& seen_policy, bool& seen_priority, bool& seen_nice) {
        if (key == "name") {
            if (seen_name)
                throw_invalid_spec("duplicate key 'name'", spec);
            seen_name = true;
            if (const auto invalid_reason = check_name(value))
                throw_invalid_spec(*invalid_reason, spec);
            name_ = to_string(value);
            return;
        }
        if (key == "cpus") {
            if (seen_cpus)
                throw_invalid_spec("duplicate key 'cpus'", spec);
            seen_cpus = true;
            cpus_ = parse_cpu_list(value, spec);
            return;
        }
        if (key == "policy") {
            if (seen_policy)
                throw_invalid_spec("duplicate key 'policy'", spec);
            seen_policy = true;
            policy_ = parse_policy(value, spec);
            return;
        }
        if (key == "priority") {
            if (seen_priority)
                throw_invalid_spec("duplicate key 'priority'", spec);
            seen_priority = true;
            priority_ = parse_int(value, key, spec);
            return;
        }
        if (key == "nice") {
            if (seen_nice)
                throw_invalid_spec("duplicate key 'nice'", spec);
            seen_nice = true;
            nice_ = parse_int(value, key, spec);
            return;
        }
        throw_invalid_spec("unknown key '" + to_string(key) + "'", spec);
    }

    static auto parse_policy(std::string_view value, std::string_view spec) -> int {
        if (value == "other")
            return SCHED_OTHER;
        if (value == "batch")
            return SCHED_BATCH;
        if (value == "idle")
            return SCHED_IDLE;
        if (value == "fifo")
            return SCHED_FIFO;
        if (value == "rr")
            return SCHED_RR;
        throw_invalid_spec("unknown policy", spec);
    }

    static auto parse_cpu_list(std::string_view value, std::string_view spec) -> cpu_set_t {
        cpu_set_t cpus;
        CPU_ZERO(&cpus);

        bool has_any_cpu = false;
        while (true) {
            const auto separator = value.find(',');
            const auto token = trim(value.substr(0, separator));
            if (token.empty())
                throw_invalid_spec("empty cpu list token", spec);

            const auto dash = token.find('-');
            if (dash == std::string_view::npos) {
                const int cpu = parse_int(token, "cpus", spec);
                set_cpu_range(cpus, cpu, cpu, spec);
            } else {
                if (token.find('-', dash + 1) != std::string_view::npos)
                    throw_invalid_spec("invalid cpu range token", spec);
                const int first_cpu = parse_int(trim(token.substr(0, dash)), "cpus", spec);
                const int last_cpu = parse_int(trim(token.substr(dash + 1)), "cpus", spec);
                set_cpu_range(cpus, first_cpu, last_cpu, spec);
            }

            has_any_cpu = true;
            if (separator == std::string_view::npos)
                break;
            value.remove_prefix(separator + 1);
        }

        if (!has_any_cpu)
            throw_invalid_spec("empty cpu list", spec);
        return cpus;
    }

    static void set_cpu_range(cpu_set_t& cpus, int first_cpu, int last_cpu, std::string_view spec) {
        if (first_cpu < 0 || last_cpu < 0 || first_cpu > last_cpu)
            throw_invalid_spec("invalid cpu range", spec);
        if (last_cpu >= CPU_SETSIZE)
            throw_invalid_spec("cpu index exceeds CPU_SETSIZE", spec);

        for (int cpu = first_cpu; cpu <= last_cpu; ++cpu)
            CPU_SET(cpu, &cpus);
    }

    void validate(std::string_view spec) const {
        if (priority_ && !policy_)
            throw_invalid_spec("priority requires policy", spec);

        if (policy_) {
            const bool is_realtime = *policy_ == SCHED_FIFO || *policy_ == SCHED_RR;
            if (is_realtime) {
                if (!priority_)
                    throw_invalid_spec("realtime policy requires priority", spec);
                if (nice_)
                    throw_invalid_spec("realtime policy cannot use nice", spec);

                const int min_priority = sched_get_priority_min(*policy_);
                const int max_priority = sched_get_priority_max(*policy_);
                if (*priority_ < min_priority || *priority_ > max_priority)
                    throw_invalid_spec("priority out of range for policy", spec);
            } else if (priority_) {
                throw_invalid_spec("non-realtime policy cannot use priority", spec);
            }
        }

        if (nice_ && (*nice_ < -20 || *nice_ > 19))
            throw_invalid_spec("nice out of range", spec);
    }

    std::optional<std::string> name_;
    std::optional<cpu_set_t> cpus_;
    std::optional<int> policy_;
    std::optional<int> priority_;
    std::optional<int> nice_;
};

} // namespace rmcs_utility
