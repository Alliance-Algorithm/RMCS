#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include <chrono>
#include <cstdint>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <typeinfo>
#include <utility>
#include <vector>

namespace rmcs_core::debug {

// Records selected executor interfaces to CSV at a fixed tick divider.
// Supported interface types: double, int, uint8, uint32, uint64, bool,
// Eigen::Vector3d (columns x,y,z) and Eigen::Quaterniond (columns w,x,y,z).
// Multi-component signals expand into "<signal>/<component>" columns.
class ValueCollector
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    ValueCollector()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {}

    auto before_pairing(const OutputInfoMap& output_map) -> void override {
        node::param("csv_path", csv_path_);

        {
            constexpr auto kPlaceholder = std::string_view{"<timestamp>"};
            const auto pos = csv_path_.find(kPlaceholder);
            if (pos != std::string::npos) {
                const auto now = std::chrono::system_clock::now();
                const auto time = std::chrono::system_clock::to_time_t(now);

                auto ss = std::ostringstream{};
                ss << std::put_time(std::localtime(&time), "%Y-%m-%d_%H-%M-%S");
                csv_path_.replace(pos, kPlaceholder.size(), ss.str());
            }
        }

        node::param("signals", signal_names_);
        node::param("write_interval", write_interval_);
        node::param("flush_interval", flush_interval_);

        if (csv_path_.empty() || signal_names_.empty())
            return;

        for (const auto& name : signal_names_) {
            const auto it = output_map.find(name);
            if (it == output_map.end()) {
                node::error("signal '{}' not found", name);
                continue;
            }
            if (it->second.kind != rmcs_executor::InterfaceKind::Normal) {
                node::error("signal '{}' is an event interface", name);
                continue;
            }
            if (!register_signal(name, it->second.type.get()))
                node::error("signal '{}' has an unsupported type", name);
        }

        if (units_.empty())
            return;

        csv_file_.open(csv_path_, std::ios::out | std::ios::trunc);
        if (!csv_file_.is_open()) {
            node::error("failed to open {}", csv_path_);
            return;
        }
        csv_file_ << std::setprecision(10);

        csv_file_ << "index";
        for (const auto& unit : units_)
            for (const auto& column : unit.columns)
                csv_file_ << "," << column;
        csv_file_ << "\n";
        csv_file_.flush();

        auto columns = std::size_t{0};
        for (const auto& unit : units_)
            columns += unit.columns.size();
        node::info(
            "collecting {} signals ({} columns) to {}", units_.size(), columns, csv_path_);
    }

    auto update() -> void override {
        if (units_.empty() || !csv_file_.is_open())
            return;

        if (tick_++ % write_interval_ != 0)
            return;

        csv_file_ << sample_count_++;
        for (const auto& unit : units_)
            unit.append(csv_file_);
        csv_file_ << "\n";

        if (sample_count_ % flush_interval_ == 0)
            csv_file_.flush();
    }

private:
    struct SignalUnit {
        std::vector<std::string> columns;
        std::function<void(std::ostream&)> append;
    };

    template <typename T, typename Writer>
    auto register_typed(
        const std::string& name, const std::type_info& type, std::vector<std::string> columns,
        Writer writer) -> bool {
        if (type != typeid(T))
            return false;

        auto holder = std::make_shared<InputInterface<T>>();
        register_input(name, *holder);
        const auto* raw = holder.get();
        keepalive_.push_back(holder);
        units_.push_back(
            SignalUnit{std::move(columns), [raw, writer](std::ostream& os) { writer(os, **raw); }});
        return true;
    }

    auto register_signal(const std::string& name, const std::type_info& type) -> bool {
        if (register_typed<double>(name, type, {name}, [](std::ostream& os, double value) {
                os << "," << value;
            }))
            return true;
        if (register_typed<int>(name, type, {name}, [](std::ostream& os, int value) {
                os << "," << value;
            }))
            return true;
        if (register_typed<std::uint8_t>(name, type, {name}, [](std::ostream& os, std::uint8_t v) {
                os << "," << static_cast<unsigned>(v);
            }))
            return true;
        if (register_typed<std::uint32_t>(name, type, {name}, [](std::ostream& os, std::uint32_t v) {
                os << "," << v;
            }))
            return true;
        if (register_typed<std::uint64_t>(name, type, {name}, [](std::ostream& os, std::uint64_t v) {
                os << "," << v;
            }))
            return true;
        if (register_typed<bool>(name, type, {name}, [](std::ostream& os, bool value) {
                os << "," << (value ? 1 : 0);
            }))
            return true;
        if (register_typed<Eigen::Vector3d>(
                name, type, {name + "/x", name + "/y", name + "/z"},
                [](std::ostream& os, const Eigen::Vector3d& value) {
                    os << "," << value.x() << "," << value.y() << "," << value.z();
                }))
            return true;
        if (register_typed<Eigen::Quaterniond>(
                name, type, {name + "/w", name + "/x", name + "/y", name + "/z"},
                [](std::ostream& os, const Eigen::Quaterniond& value) {
                    os << "," << value.w() << "," << value.x() << "," << value.y() << ","
                       << value.z();
                }))
            return true;
        return false;
    }

    std::string csv_path_;
    std::vector<std::string> signal_names_;
    int write_interval_ = 1;
    int flush_interval_ = 100;

    std::vector<SignalUnit> units_;
    // Keeps every registered input alive; SignalUnit lambdas point into these.
    std::vector<std::shared_ptr<void>> keepalive_;
    std::ofstream csv_file_;

    int tick_ = 0;
    int sample_count_ = 0;
};

} // namespace rmcs_core::debug

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::debug::ValueCollector, rmcs_executor::Component)
