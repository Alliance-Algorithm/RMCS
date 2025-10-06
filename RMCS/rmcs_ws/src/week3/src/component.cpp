#include <rmcs_executor/component.hpp>

class Component final : public rmcs_executor::Component {
public:
    void update() override {
    }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(Component, rmcs_executor::Component)