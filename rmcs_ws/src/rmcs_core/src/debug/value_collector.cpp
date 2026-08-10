#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace rmcs_core::debug {

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
            if (it->second.type.get() != typeid(double)) {
                node::error("signal '{}' type is not double", name);
                continue;
            }

            auto unit = std::make_unique<SignalUnit>();
            unit->name = name;
            register_input(name, unit->value);
            units_.push_back(std::move(unit));
        }

        if (units_.empty())
            return;

        csv_file_.open(csv_path_, std::ios::out | std::ios::trunc);
        if (!csv_file_.is_open()) {
            node::error("failed to open {}", csv_path_);
            return;
        }

        csv_file_ << "index";
        for (const auto& unit : units_)
            csv_file_ << "," << unit->name;
        csv_file_ << "\n";
        csv_file_.flush();

        node::info("collecting {} signals to {}", units_.size(), csv_path_);
    }

    auto update() -> void override {
        if (units_.empty() || !csv_file_.is_open())
            return;

        if (tick_++ % write_interval_ != 0)
            return;

        csv_file_ << sample_count_++;
        for (const auto& unit : units_)
            csv_file_ << "," << *unit->value;
        csv_file_ << "\n";

        if (sample_count_ % flush_interval_ == 0)
            csv_file_.flush();
    }

private:
    struct SignalUnit {
        std::string name;
        InputInterface<double> value;
    };

    std::string csv_path_;
    std::vector<std::string> signal_names_;
    int write_interval_ = 1;
    int flush_interval_ = 100;

    std::vector<std::unique_ptr<SignalUnit>> units_;
    std::ofstream csv_file_;

    int tick_ = 0;
    int sample_count_ = 0;
};

} // namespace rmcs_core::debug

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::debug::ValueCollector, rmcs_executor::Component)
