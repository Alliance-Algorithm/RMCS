#include "visualization.hpp"
#include "modules/debug/visualization/stream_session.hpp"

#include "utility/image.details.hpp"
#include "utility/logging/printer.hpp"
#include "utility/serializable.hpp"

#include <fstream>

using namespace rmcs::kernel;

constexpr std::array kVideoTypes {
    "RTP_JEPG",
    "RTP_H264",
};

struct Visualization::Impl {
    using SessionConfig = debug::StreamSession::Config;
    using NormalResult  = std::expected<void, std::string>;

    struct Config : util::Serializable {

        util::integer_t framerate = 80;

        util::string_t monitor_host = "localhost";
        util::string_t monitor_port = "5000";

        util::string_t stream_type = "RTP_JEPG";

        static constexpr auto metas = std::tuple {
            &Config::framerate,
            "framerate",
            &Config::monitor_host,
            "monitor_host",
            &Config::monitor_port,
            "monitor_port",
            &Config::stream_type,
            "stream_type",
        };
    };

    Printer log { "visualization" };

    std::unique_ptr<debug::StreamSession> session;
    SessionConfig session_config;

    bool is_initialized  = false;
    bool size_determined = false;

    Impl() noexcept { session = std::make_unique<debug::StreamSession>(); }

    auto initialize(const YAML::Node& yaml) noexcept -> NormalResult {
        auto config = Config {};
        auto result = config.serialize(yaml);
        if (!result.has_value()) {
            return std::unexpected { result.error() };
        }

        session_config.target.host = config.monitor_host;
        session_config.target.port = config.monitor_port;
        session_config.format.hz   = static_cast<int>(config.framerate);

        if (config.stream_type == kVideoTypes[0]) {
            session_config.type = debug::StreamType::RTP_JEPG;
        } else if (config.stream_type == kVideoTypes[1]) {
            session_config.type = debug::StreamType::RTP_H264;
        } else {
            return std::unexpected { "Unknown video type: " + config.stream_type };
        }
        is_initialized = true;
        return {};
    }

    auto initialized() const noexcept { return is_initialized; }

    auto send_image(const Image& image) noexcept -> bool {
        if (!is_initialized) return false;

        const auto& mat = image.details().mat;

        if (!size_determined) {
            session_config.format.w = mat.cols;
            session_config.format.h = mat.rows;

            { // open session
                auto ret = session->open(session_config);
                if (!ret) {
                    log.error("Failed to open visualization session");
                    log.error("  e: {}", ret.error());
                    return false;
                }
                log.info("Visualization session is opened");
            }
            { // write SDP
                auto ret = session->session_description_protocol();
                if (!ret) {
                    log.error("Failed to get description protocol");
                    log.error("  e: {}", ret.error());
                    return false;
                }

                const auto output_location = "/tmp/auto_aim.sdp";
                if (auto ofstream = std::ofstream { output_location }) {
                    ofstream << ret.value();
                    log.info("Sdp has been written to: {}", output_location);
                } else {
                    log.error("Failed to write sdp: {}", output_location);
                    return false;
                }
            }
            size_determined = true;
        }
        if (!session->opened()) return false;

        return session->push_frame(mat);
    }
};

auto Visualization::initialize(const YAML::Node& yaml) noexcept
    -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto Visualization::initialized() const noexcept -> bool { return pimpl->initialized(); }

auto Visualization::send_image(const Image& image) noexcept -> bool {
    return pimpl->send_image(image);
}

Visualization::Visualization() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Visualization::~Visualization() noexcept = default;
