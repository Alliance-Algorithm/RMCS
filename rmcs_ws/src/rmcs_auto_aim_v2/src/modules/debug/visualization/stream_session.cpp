#include "stream_session.hpp"

#include <functional>
#include <memory>
#include <stop_token>
#include <string>
#include <thread>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>

#include <boost/lockfree/spsc_queue.hpp>
#include <opencv2/core/mat.hpp>

using namespace rmcs::debug;

struct StreamSession::Impl final {
public:
    auto session_description_protocol() const noexcept -> std::expected<std::string, std::string> {

        if (!context || !context->opened()) {
            return std::unexpected { "Stream context is not initialized" };
        }

        const auto local_ipv4 = get_same_subnet_ipv4(context->stream_target().host);
        if (!local_ipv4) {
            return std::unexpected { std::format("Local IPv4 not found: {}", local_ipv4.error()) };
        }

        const auto result = context->session_description_protocol(*local_ipv4);
        if (!result) {
            return std::unexpected { result.error() };
        }

        return *result;
    }

    auto initialize(StreamType type, const StreamTarget& target, const VideoFormat& format) noexcept
        -> void {
        context  = std::make_unique<StreamContext>(type, format, target);
        notifier = [](const std::string&) { };
        thread   = std::make_unique<std::jthread>(
            [this](const std::stop_token& token) { streaming_thread(token); });
    }

    auto open() noexcept -> std::expected<void, std::string> { return context->open(); }

    auto opened() const noexcept { return context && context->opened(); }

    auto push_frame(FrameRef frame) noexcept -> bool { return buffer.push(frame); }

    void set_notifier(std::function<void(const std::string&)> f) { notifier = std::move(f); }

private:
    auto streaming_thread(const std::stop_token& token) noexcept -> void {
        notifier("Streaming thread starts");

        while (!token.stop_requested()) {

            auto current_frame = cv::Mat {};
            if (buffer.pop(current_frame)) {
                context->write(current_frame);
            }
            std::this_thread::yield();
        }
        notifier("Streaming thread stops");
    }

    struct NetworkInfo {
        in_addr_t address;
        in_addr_t netmask;
    };

    static auto get_network_info() -> std::expected<std::vector<NetworkInfo>, std::error_code> {
        ifaddrs* if_list;
        if (getifaddrs(&if_list) == -1) {
            return std::unexpected(std::error_code(errno, std::generic_category()));
        }

        std::vector<NetworkInfo> network_info_list;
        for (ifaddrs* if_ptr = if_list; if_ptr; if_ptr = if_ptr->ifa_next) {
            if (if_ptr->ifa_addr && if_ptr->ifa_addr->sa_family == AF_INET) {
                const auto* sa_in = reinterpret_cast<sockaddr_in*>(if_ptr->ifa_addr);
                const auto* sn_in = reinterpret_cast<sockaddr_in*>(if_ptr->ifa_netmask);
                network_info_list.push_back({ sa_in->sin_addr.s_addr, sn_in->sin_addr.s_addr });
            }
        }
        freeifaddrs(if_list);
        return network_info_list;
    }
    static auto get_same_subnet_ipv4(std::string_view target_ip_str)
        -> std::expected<std::string, std::string> {

        auto target_addr = in_addr {};
        if (inet_pton(AF_INET, target_ip_str.data(), &target_addr) != 1) {
            return std::unexpected(std::format("Invalid target IP address: {}", target_ip_str));
        }

        const auto network_info_expected = get_network_info();
        if (!network_info_expected) {
            return std::unexpected(std::format(
                "Failed to get network info: {}", network_info_expected.error().message()));
        }

        const auto& network_info_list = network_info_expected.value();
        const auto& target_addr_bin   = target_addr.s_addr;

        for (const auto& info : network_info_list) {
            if ((info.address & info.netmask) == (target_addr_bin & info.netmask)) {
                char local_ip[INET_ADDRSTRLEN];
                if (inet_ntop(AF_INET, &info.address, local_ip, sizeof(local_ip))) {
                    return std::string(local_ip);
                }
            }
        }

        return std::unexpected(
            std::format("Could not find a local IP in the same subnet as {}", target_ip_str));
    }

private:
    std::unique_ptr<StreamContext> context;
    std::unique_ptr<std::jthread> thread;

    static constexpr auto buffer_capacity = std::size_t { 100 };
    boost::lockfree::spsc_queue<cv::Mat, boost::lockfree::capacity<buffer_capacity>> buffer;

    std::function<void(const std::string&)> notifier;
};

StreamSession::StreamSession() noexcept
    : pimpl { std::make_unique<Impl>() } { }

StreamSession::~StreamSession() noexcept = default;

auto StreamSession::set_notifier(std::function<void(const std::string&)> f) noexcept -> void {
    pimpl->set_notifier(std::move(f));
}
auto StreamSession::open(const Config& config) noexcept -> std::expected<void, std::string> {
    pimpl->initialize(config.type, config.target, config.format);
    return pimpl->open();
}
auto StreamSession::opened() const noexcept -> bool { return pimpl->opened(); }

auto StreamSession::push_frame(FrameRef frame) noexcept -> bool { return pimpl->push_frame(frame); }

auto StreamSession::session_description_protocol() const noexcept
    -> std::expected<std::string, std::string> {
    return pimpl->session_description_protocol();
}
