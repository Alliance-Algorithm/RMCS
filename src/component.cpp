#include "modules/debug/framerate.hpp"
#include "utility/node.hpp"
#include "utility/shared/context.hpp"
#include "utility/shared/interprocess.hpp"

#include <rmcs_executor/component.hpp>

namespace rmcs {
using Component = rmcs_executor::Component;
using Node      = util::Node;
using Client    = shm::Client<shared::Context>::Recv;

class AutoAimComponent final : public Component, Node {
public:
    explicit AutoAimComponent() noexcept
        : Node { get_component_name(), util::options } {

        using namespace std::chrono_literals;
        framerate.set_intetval(2s);

        if (!client.open(shared::id)) {
            error("Failed to open shared memory");
        }
    }

    auto update() -> void override {
        if (client.opened() == false) {
            client.open(shared::id);
        } else if (client.is_updated() && framerate.tick()) {

            auto context = shared::Context {};
            client.recv(context);

            auto timestamp = context.timestamp;
            auto now       = shared::Clock::now();

            using Milli   = std::chrono::duration<double, std::milli>;
            auto interval = Milli { now - timestamp };

            info("Client recv, delay: {:.3}ms, hz: {}", interval.count(), framerate.fps());
        }
    }

private:
    Client client;
    FramerateCounter framerate;
};

} // namespace rmcs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimComponent, rmcs_executor::Component)
