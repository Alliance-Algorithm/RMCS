#pragma once

#include <memory>
namespace world_exe::tongji {

class AutoAimSystem final {
public:
    static void build(bool debug);

    explicit AutoAimSystem(const bool& debug);
    ~AutoAimSystem();

    AutoAimSystem(const AutoAimSystem&)            = delete;
    AutoAimSystem& operator=(const AutoAimSystem&) = delete;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    static std::unique_ptr<AutoAimSystem> v2;
};
}