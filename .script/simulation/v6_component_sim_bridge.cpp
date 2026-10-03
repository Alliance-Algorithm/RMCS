#include <array>
#include <bit>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <typeinfo>
#include <utility>
#include <vector>

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/chassis/wheel_leg_chassis_controller.cpp"
#include "rl_controller.hpp"

// Use the same executor friendship as the production-component ABI tests. Only
// registered interfaces are visible; controller algorithms remain unchanged.
namespace rmcs_executor {
class Executor {
public:
    template <typename T>
    static void bind(Component& component, const std::string& name, T& value) {
        for (auto& port : component.input_list_) {
            if (port.name == name) {
                if (port.type != typeid(T) || port.kind != InterfaceKind::Normal)
                    throw std::runtime_error("Wrong input type: " + name);
                port.bind(port.binding, &value);
                return;
            }
        }
        throw std::runtime_error("Input is not registered: " + name);
    }

    template <typename T>
    static T* find_output(Component& component, const std::string& name) {
        for (auto& port : component.output_list_) {
            if (port.name == name) {
                if (port.type != typeid(T) || port.kind != InterfaceKind::Normal)
                    throw std::runtime_error("Wrong output type: " + name);
                return static_cast<T*>(port.binding);
            }
        }
        return nullptr;
    }

    template <typename T>
    static T& output(Component& component, const std::string& name) {
        if (const auto value = find_output<T>(component, name))
            return *value;
        throw std::runtime_error("Output is not registered: " + name);
    }
};
} // namespace rmcs_executor

namespace {

using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using Ports = rmcs_executor::Executor;
using Chassis = rmcs_core::controller::chassis::WheelLegChassisController;
using RlController = rmcs::rl::RlController;
using Direction = rmcs_description::BaseLink::DirectionVector;

struct Options {
    bool simulation_only = false;
    std::string profile, model, socket, model_sha256;
    std::vector<std::string> parameters;
};

std::string file_sha256(const std::string& path) {
    std::ifstream file{path, std::ios::binary};
    const std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> digest{
        EVP_MD_CTX_new(), EVP_MD_CTX_free};
    if (!file || !digest || EVP_DigestInit_ex(digest.get(), EVP_sha256(), nullptr) != 1)
        throw std::runtime_error("Cannot initialize model digest");
    std::array<char, 8192> buffer{};
    while (file.read(buffer.data(), buffer.size()) || file.gcount())
        if (EVP_DigestUpdate(digest.get(), buffer.data(), file.gcount()) != 1)
            throw std::runtime_error("Cannot update model digest");
    std::array<unsigned char, EVP_MAX_MD_SIZE> bytes{};
    unsigned int size = 0;
    if (!file.eof() || EVP_DigestFinal_ex(digest.get(), bytes.data(), &size) != 1 || size != 32)
        throw std::runtime_error("Cannot finish model digest");
    constexpr std::string_view hex = "0123456789abcdef";
    std::string result;
    for (unsigned int i = 0; i < size; ++i) {
        result.push_back(hex[bytes[i] >> 4]);
        result.push_back(hex[bytes[i] & 15]);
    }
    return result;
}

Options parse_options(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string_view argument{argv[i]};
        if (argument == "--simulation-only") {
            options.simulation_only = true;
            continue;
        }
        if (i + 1 >= argc)
            throw std::invalid_argument("Missing value after " + std::string{argument});
        const std::string value{argv[++i]};
        if (argument == "--profile")
            options.profile = value;
        else if (argument == "--model")
            options.model = value;
        else if (argument == "--socket")
            options.socket = value;
        else if (argument == "--param")
            options.parameters.push_back(value);
        else
            throw std::invalid_argument("Unknown option: " + std::string{argument});
    }
    if (!options.simulation_only || options.profile.empty() || options.model.empty()
        || options.socket.empty())
        throw std::invalid_argument(
            "Required: --simulation-only --profile YAML --model ONNX --socket PATH "
            "[--param name:=value]");
    if (!std::filesystem::is_regular_file(options.profile)
        || !std::filesystem::is_regular_file(options.model))
        throw std::invalid_argument("The profile and model must be existing files");
    options.profile = std::filesystem::absolute(options.profile).lexically_normal().string();
    options.model = std::filesystem::absolute(options.model).lexically_normal().string();
    options.model_sha256 = file_sha256(options.model);
    return options;
}

Json public_parameters(const rclcpp::Node& node, std::initializer_list<const char*> names) {
    Json result = Json::object();
    for (const auto* name : names) {
        if (!node.has_parameter(name))
            continue;
        const auto parameter = node.get_parameter(name);
        switch (parameter.get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL: result[name] = parameter.as_bool(); break;
        case rclcpp::ParameterType::PARAMETER_INTEGER: result[name] = parameter.as_int(); break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE: result[name] = parameter.as_double(); break;
        case rclcpp::ParameterType::PARAMETER_STRING: result[name] = parameter.as_string(); break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
            result[name] = parameter.as_double_array();
            break;
        default: throw std::runtime_error("Unexpected parameter type: " + std::string{name});
        }
    }
    return result;
}

void initialize_ros(const Options& options) {
    std::vector<std::string> args{"v6_component_sim_bridge", "--ros-args",  "--params-file",
                                  options.profile,           "--log-level", "error"};
    const auto add_parameter = [&args](const std::string& parameter) {
        args.push_back("-p");
        args.push_back(parameter);
    };
    // These readiness flags describe the simulated sensors and CAD mechanism.
    // They do not write the profile or affect any running hardware process.
    for (const auto& parameter : std::vector<std::string>{
             "rl_model_path:=" + std::filesystem::absolute(options.model).string(),
             "policy_profile:=v6_flat_14020", "calibration_ready:=true", "soft_limits_ready:=true",
             "imu_alignment_ready:=true", "recovery_enabled:=false", "auto_enter_rl:=false",
             "hinge_min:=[-0.140638855,-0.140638855]", "hinge_max:=[1.329574849,1.329574849]"})
        add_parameter(parameter);
    for (const auto& parameter : options.parameters)
        add_parameter(parameter);
    std::vector<const char*> argv;
    for (const auto& argument : args)
        argv.push_back(argument.c_str());
    rclcpp::init(static_cast<int>(argv.size()), argv.data());
}

template <std::size_t Size>
std::array<double, Size> read_array(const Json& request, const char* name) {
    const auto& values = request.at(name);
    if (!values.is_array() || values.size() != Size)
        throw std::invalid_argument(std::string{name} + " has the wrong number of values");
    std::array<double, Size> result{};
    for (std::size_t i = 0; i < Size; ++i) {
        result[i] = values.at(i).get<double>();
        if (!std::isfinite(result[i]))
            throw std::invalid_argument(std::string{name} + " must be finite");
    }
    return result;
}

rmcs_msgs::Switch read_switch(const Json& request, const char* name) {
    const auto value = request.at(name).get<int>();
    if (value < 0 || value > 3)
        throw std::invalid_argument(std::string{name} + " must be 0, 1, 2 or 3");
    return static_cast<rmcs_msgs::Switch>(value);
}

struct Snapshot {
    std::array<double, 6> q, dq, feedback_torque{}, max_torque{40., 40., 40., 40., 4.5, 4.5};
    std::array<double, 4> orientation;
    std::array<double, 3> gyro, acceleration;
    std::array<double, 2> right_stick, left_stick;
    std::array<int, 4> faults{};
    rmcs_msgs::Switch left_switch, right_switch;
    rmcs_msgs::Keyboard keyboard = rmcs_msgs::Keyboard::zero();
    double rotary_knob = 0.;
    bool remote_fresh = true, feedback_fresh = true, dm_control_ready = true;

    explicit Snapshot(const Json& request)
        : q{read_array<6>(request, "q_api")}
        , dq{read_array<6>(request, "dq_api")}
        , orientation{read_array<4>(request, "orientation_wxyz")}
        , gyro{read_array<3>(request, "gyro")}
        , acceleration{read_array<3>(request, "acceleration")}
        , right_stick{read_array<2>(request, "right_stick")}
        , left_stick{read_array<2>(request, "left_stick")}
        , left_switch{read_switch(request, "left_switch")}
        , right_switch{read_switch(request, "right_switch")} {
        if (request.contains("feedback_torque"))
            feedback_torque = read_array<6>(request, "feedback_torque");
        if (request.contains("max_torque"))
            max_torque = read_array<6>(request, "max_torque");
        if (request.contains("fault_codes")) {
            const auto& values = request.at("fault_codes");
            if (!values.is_array() || values.size() != faults.size())
                throw std::invalid_argument("fault_codes must contain four integers");
            for (std::size_t i = 0; i < faults.size(); ++i)
                faults[i] = values.at(i).get<int>();
        }
        keyboard = std::bit_cast<rmcs_msgs::Keyboard>(request.value("keyboard", std::uint16_t{0}));
        rotary_knob = request.value("rotary_knob", 0.);
        if (!std::isfinite(rotary_knob))
            throw std::invalid_argument("rotary_knob must be finite");
        remote_fresh = request.value("remote_fresh", true);
        feedback_fresh = request.value("feedback_fresh", true);
        dm_control_ready = request.value("dm_control_ready", true);
    }
};

class Session {
public:
    Session() {
        rmcs_executor::Component::initializing_component_name = "wheel_leg_chassis_controller";
        chassis_ = std::make_unique<Chassis>();
        rmcs_executor::Component::initializing_component_name = "rl_controller";
        controller_ = std::make_unique<RlController>();
        takeover_blend_fraction_ =
            Ports::find_output<double>(*controller_, "/wheel_leg/rl/v6_takeover/blend_fraction");
        bind_chassis_("/remote/joystick/right", right_stick_);
        bind_chassis_("/remote/joystick/left", left_stick_);
        bind_chassis_("/remote/switch/right", right_switch_);
        bind_chassis_("/remote/switch/left", left_switch_);
        bind_chassis_("/wheel_leg/dr16_fresh", remote_fresh_);
        bind_chassis_("/remote/rotary_knob", rotary_knob_);
        bind_chassis_("/remote/keyboard", keyboard_);
        connect_<Direction>("/chassis/control_velocity");
        connect_<double>("/chassis/control_height");
        connect_<int>("/chassis/control_state");
        connect_<std::size_t>("/chassis/reset_count");
        connect_<rmcs_msgs::ChassisMode>("/chassis/control_mode");
        connect_<bool>("/chassis/jump_request");
        connect_<double>("/chassis/jump_apex_delta");
        bind_("/predefined/update_count", tick_);
        bind_("/predefined/update_rate", rate_);
        bind_("/predefined/timestamp", timestamp_);
        bind_("/wheel_leg/feedback_fresh", feedback_fresh_);
        bind_("/wheel_leg/dm_control_ready", dm_control_ready_);
        bind_("/wheel_leg/imu/quaternion", orientation_);
        bind_("/wheel_leg/imu/angular_velocity", gyro_);
        bind_("/wheel_leg/imu/acceleration", acceleration_);
        bind_("/wheel_leg/imu/acceleration_steady_ns", sample_ns_);
        bind_("/wheel_leg/imu/last_steady_ns", sample_ns_);
        bind_("/wheel_leg/imu/sequence", sequence_);
        bind_("/wheel_leg/imu/acceleration_sequence", sequence_);
        for (std::size_t axis = 0; axis < rmcs::rl::kMotorNames.size(); ++axis) {
            const auto prefix = std::string{"/wheel_leg/"} + rmcs::rl::kMotorNames[axis];
            bind_(prefix + "/angle", q_[axis]);
            bind_(prefix + "/velocity", dq_[axis]);
            bind_(prefix + "/torque", feedback_torque_[axis]);
            bind_(prefix + "/max_torque", max_torque_[axis]);
            bind_(prefix + "/feedback_sequence", sequence_);
            bind_(prefix + "/feedback_steady_ns", sample_ns_);
            if (axis < 4) {
                bind_(prefix + "/fault_code", faults_[axis]);
            } else {
                bind_(prefix + "/last_submitted_torque", submitted_torque_[axis - 4]);
                bind_(prefix + "/last_submitted_kind", submitted_kind_);
                bind_(prefix + "/last_submitted_steady_ns", sample_ns_);
            }
        }
        chassis_->before_updating();
        controller_->before_updating();
    }

    Json step(const Json& request) {
        const Snapshot snapshot{request};
        // Never fabricate hardware sample timestamps. Pace fast clients at the
        // 200 Hz sensor rate; slower host steps remain visible to the guard.
        std::this_thread::sleep_until(next_sample_time_);
        const auto now = Clock::now();
        next_sample_time_ = now + std::chrono::milliseconds{5};
        wall_sample_interval_ms_ =
            last_sample_time_ == Clock::time_point{}
                ? 0.
                : std::chrono::duration<double, std::milli>{now - last_sample_time_}.count();
        last_sample_time_ = now;
        sample_ns_ = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
        ++sequence_;
        q_ = snapshot.q;
        dq_ = snapshot.dq;
        feedback_torque_ = snapshot.feedback_torque;
        max_torque_ = snapshot.max_torque;
        faults_ = snapshot.faults;
        const auto& orientation = snapshot.orientation;
        orientation_ =
            Eigen::Quaterniond{orientation[0], orientation[1], orientation[2], orientation[3]};
        gyro_ << snapshot.gyro[0], snapshot.gyro[1], snapshot.gyro[2];
        acceleration_ << snapshot.acceleration[0], snapshot.acceleration[1],
            snapshot.acceleration[2];
        right_stick_ << snapshot.right_stick[0], snapshot.right_stick[1];
        left_stick_ << snapshot.left_stick[0], snapshot.left_stick[1];
        left_switch_ = snapshot.left_switch;
        right_switch_ = snapshot.right_switch;
        keyboard_ = snapshot.keyboard;
        rotary_knob_ = snapshot.rotary_knob;
        remote_fresh_ = snapshot.remote_fresh;
        feedback_fresh_ = snapshot.feedback_fresh;
        dm_control_ready_ = snapshot.dm_control_ready;
        // Five executor updates share one immutable physical sensor snapshot.
        // Production controller divisors supply 50 Hz ONNX and 200 Hz PD.
        for (int substep = 0; substep < 5; ++substep) {
            std::this_thread::sleep_until(now + std::chrono::milliseconds{substep + 1});
            ++tick_;
            timestamp_ += std::chrono::milliseconds{1};
            chassis_->update();
            controller_->update();
        }
        wall_step_duration_ms_ =
            std::chrono::duration<double, std::milli>{Clock::now() - now}.count();
        const auto result = response();
        const auto torque = result.at("torque_api").get<std::array<double, 6>>();
        submitted_torque_ = {torque[4], torque[5]};
        submitted_kind_ =
            result.at("enable_request").get<bool>() ? std::uint8_t{1} : std::uint8_t{0};
        return result;
    }

    Json response() {
        std::array<double, 6> torque{}, actions{};
        std::array<double, rmcs::rl::ObservationLayout::kSize> observation{};
        for (std::size_t axis = 0; axis < rmcs::rl::kMotorNames.size(); ++axis) {
            torque[axis] =
                output_<double>(std::string{rmcs::rl::kMotorNames[axis]} + "/control_torque");
            actions[axis] =
                output_<double>(std::string{"rl/action/"} + rmcs::rl::kMotorNames[axis]);
        }
        for (std::size_t slot = 0; slot < observation.size(); ++slot)
            observation[slot] =
                output_<double>("rl/observation/" + std::string{rmcs::rl::kObservationNames[slot]});
        const auto velocity = Ports::output<Direction>(*chassis_, "/chassis/control_velocity");
        bool effort_within_limits = true;
        for (std::size_t axis = 0; axis < torque.size(); ++axis)
            effort_within_limits = effort_within_limits && std::isfinite(torque[axis])
                                && std::abs(torque[axis]) <= max_torque_[axis] + 1e-9;
        Json result{
            {"ok", true},
            {"torque_api", torque},
            {"max_torque", max_torque_},
            {"effort_within_limits", effort_within_limits},
            {"state", output_<int>("rl/state")},
            {"enable_request", output_<bool>("enable_request")},
            {"mode",
             std::to_underlying(
                 Ports::output<rmcs_msgs::ChassisMode>(*chassis_, "/chassis/control_mode"))},
            {"requested_state", Ports::output<int>(*chassis_, "/chassis/control_state")},
            {"reset_count", Ports::output<std::size_t>(*chassis_, "/chassis/reset_count")},
            {"observation", observation},
            {"actions", actions},
            {"command_velocity", {velocity.vector.x(), velocity.vector.y(), velocity.vector.z()}},
            {"command_height", Ports::output<double>(*chassis_, "/chassis/control_height")},
            {"jump_request", Ports::output<bool>(*chassis_, "/chassis/jump_request")},
            {"update_count", tick_},
            {"sensor_sequence", sequence_},
            {"sensor_steady_ns", sample_ns_},
            {"wall_sample_interval_ms", wall_sample_interval_ms_},
            {"wall_step_duration_ms", wall_step_duration_ms_},
            {"sensor_issue", output_<int>("rl/recovery/sensor_issue")},
            {"sensor_mask", output_<int>("rl/recovery/sensor_invalid_mask")},
            {"sensors_valid", output_<bool>("rl/recovery/sensors_valid")},
            {"motor_age_ms", output_<double>("rl/recovery/motor_age_ms")},
            {"imu_age_ms", output_<double>("rl/recovery/imu_age_ms")},
            {"acceleration_age_ms", output_<double>("rl/recovery/acceleration_age_ms")},
            {"failure", output_<int>("rl/recovery/failure")},
            {"inference_us", output_<double>("rl/performance/inference_us")},
            {"pd_us", output_<double>("rl/performance/pd_us")}};
        if (takeover_blend_fraction_)
            result["v6_takeover_blend_fraction"] = *takeover_blend_fraction_;
        return result;
    }

    bool supports_takeover_blend() const { return takeover_blend_fraction_ != nullptr; }

    Json parameters() const {
        return Json{
            {"rl_controller", public_parameters(
                                  *controller_, {"policy_profile",
                                                 "rl_model_path",
                                                 "calibration_ready",
                                                 "soft_limits_ready",
                                                 "imu_alignment_ready",
                                                 "recovery_enabled",
                                                 "auto_enter_rl",
                                                 "leg_motor_to_model",
                                                 "leg_model_offsets",
                                                 "wheel_model_scale",
                                                 "nominal_model_pos",
                                                 "imu_to_base",
                                                 "hinge_coefficients",
                                                 "hinge_bias",
                                                 "hinge_min",
                                                 "hinge_max",
                                                 "hinge_margin",
                                                 "rl_inference_frequency",
                                                 "prepare_stable_seconds",
                                                 "v6_takeover_blend_seconds",
                                                 "height_transition_seconds",
                                                 "dm_feedback_position_max"})},
            {"chassis",
             public_parameters(
                 *chassis_, {"vx_max", "vy_max", "yaw_rate_max", "spin_yaw_rate", "deadzone",
                             "command_height_min", "command_height_max", "default_command_height",
                             "height_step", "angular_z_invert", "height_invert", "jump_enabled"})}};
    }

private:
    template <typename T>
    void bind_(const std::string& name, T& value) {
        Ports::bind(*controller_, name, value);
    }

    template <typename T>
    void bind_chassis_(const std::string& name, T& value) {
        Ports::bind(*chassis_, name, value);
    }

    template <typename T>
    void connect_(const std::string& name) {
        bind_(name, Ports::output<T>(*chassis_, name));
    }

    template <typename T>
    const T& output_(const std::string& name) {
        return Ports::output<T>(*controller_, "/wheel_leg/" + name);
    }

    std::unique_ptr<Chassis> chassis_;
    std::unique_ptr<RlController> controller_;
    const double* takeover_blend_fraction_ = nullptr;
    std::size_t tick_ = 0;
    double rate_ = 1000.;
    Clock::time_point timestamp_ = Clock::now(), next_sample_time_{}, last_sample_time_{};
    double wall_sample_interval_ms_ = 0., wall_step_duration_ms_ = 0.;
    std::uint64_t sequence_ = 0, sample_ns_ = 0;
    std::array<double, 6> q_{}, dq_{}, feedback_torque_{},
        max_torque_{40., 40., 40., 40., 4.5, 4.5};
    std::array<int, 4> faults_{};
    std::array<double, 2> submitted_torque_{};
    std::uint8_t submitted_kind_ = 0;
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
    Eigen::Vector3d gyro_ = Eigen::Vector3d::Zero(), acceleration_{0., 0., 9.81};
    Eigen::Vector2d right_stick_ = Eigen::Vector2d::Zero(), left_stick_ = Eigen::Vector2d::Zero();
    rmcs_msgs::Switch left_switch_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch right_switch_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard keyboard_ = rmcs_msgs::Keyboard::zero();
    double rotary_knob_ = 0.;
    bool remote_fresh_ = true, feedback_fresh_ = true, dm_control_ready_ = true;
};

class FileDescriptor {
public:
    explicit FileDescriptor(int value)
        : value_{value} {
        if (value < 0)
            throw std::runtime_error(std::strerror(errno));
    }
    ~FileDescriptor() { ::close(value_); }
    FileDescriptor(const FileDescriptor&) = delete;
    FileDescriptor& operator=(const FileDescriptor&) = delete;
    int get() const { return value_; }

private:
    int value_;
};

void send_response(int socket, const Json& response) {
    const auto data = response.dump() + '\n';
    std::size_t sent = 0;
    while (sent < data.size()) {
        const auto count = ::send(socket, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
        if (count < 0 && errno == EINTR)
            continue;
        if (count <= 0)
            throw std::runtime_error("Socket write failed");
        sent += static_cast<std::size_t>(count);
    }
}

bool serve_connection(int socket, const Options& options) {
    auto session = std::make_unique<Session>();
    std::string pending;
    std::array<char, 8192> buffer{};
    for (;;) {
        const auto count = ::recv(socket, buffer.data(), buffer.size(), 0);
        if (count < 0 && errno == EINTR)
            continue;
        if (count <= 0)
            return true;
        pending.append(buffer.data(), static_cast<std::size_t>(count));
        if (pending.size() > 65536)
            throw std::runtime_error("Request exceeds 64 KiB");
        for (;;) {
            const auto end = pending.find('\n');
            if (end == std::string::npos)
                break;
            const auto line = pending.substr(0, end);
            pending.erase(0, end + 1);
            try {
                const auto request = Json::parse(line);
                const auto operation = request.at("op").get<std::string>();
                if (operation == "step") {
                    send_response(socket, session->step(request));
                } else if (operation == "reset") {
                    session = std::make_unique<Session>();
                    send_response(socket, session->response());
                } else if (operation == "hello") {
                    auto response = session->response();
                    response["protocol"] = "rmcs_v6_component_sim_v1";
                    response["simulation_only"] = true;
                    response["profile_path"] = options.profile;
                    response["model_path"] = options.model;
                    response["model_sha256"] = options.model_sha256;
                    response["parameters"] = session->parameters();
                    response["v6_takeover_blend_supported"] = session->supports_takeover_blend();
                    response["policy_profile"] =
                        response["parameters"]["rl_controller"].at("policy_profile");
                    response["axis_order"] = rmcs::rl::kMotorNames;
                    response["executor_hz"] = 1000;
                    response["physics_hz"] = 200;
                    response["inference_hz"] = 50;
                    response["pd_hz"] = 200;
                    send_response(socket, response);
                } else if (operation == "close" || operation == "shutdown") {
                    send_response(socket, Json{{"ok", true}});
                    return operation != "shutdown";
                } else {
                    throw std::invalid_argument("Unknown operation: " + operation);
                }
            } catch (const std::exception& error) {
                send_response(socket, Json{{"ok", false}, {"error", error.what()}});
            }
        }
    }
}

void serve(const Options& options) {
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if (options.socket.size() >= sizeof(address.sun_path))
        throw std::invalid_argument("Socket path is too long");
    if (std::filesystem::exists(options.socket))
        throw std::invalid_argument("Socket path already exists: " + options.socket);
    std::memcpy(address.sun_path, options.socket.c_str(), options.socket.size() + 1);
    FileDescriptor listener{::socket(AF_UNIX, SOCK_STREAM, 0)};
    if (::bind(listener.get(), reinterpret_cast<const sockaddr*>(&address), sizeof(address)) < 0)
        throw std::runtime_error("Cannot bind socket: " + std::string{std::strerror(errno)});
    struct SocketCleanup {
        std::string path;
        ~SocketCleanup() { ::unlink(path.c_str()); }
    } cleanup{options.socket};
    if (::chmod(options.socket.c_str(), 0600) < 0 || ::listen(listener.get(), 1) < 0)
        throw std::runtime_error("Cannot listen on socket");
    std::cerr << "Simulation-only production component bridge: " << options.socket << '\n';
    for (;;) {
        const auto descriptor = ::accept(listener.get(), nullptr, nullptr);
        if (descriptor < 0 && errno == EINTR)
            continue;
        FileDescriptor connection{descriptor};
        try {
            if (!serve_connection(connection.get(), options))
                return;
        } catch (const std::exception& error) {
            std::cerr << "Simulation connection ended: " << error.what() << '\n';
        }
    }
}

} // namespace

int main(int argc, char** argv) {
    try {
        const auto options = parse_options(argc, argv);
        initialize_ros(options);
        serve(options);
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "v6_component_sim_bridge: " << error.what() << '\n';
        if (rclcpp::ok())
            rclcpp::shutdown();
        return 1;
    }
}
