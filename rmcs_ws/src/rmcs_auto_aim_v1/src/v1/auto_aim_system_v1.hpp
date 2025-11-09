#pragma once

#include <memory>
namespace world_exe::v1 {
class SystemV1 final{
public:
    static void build(const bool& debug);

    SystemV1(const bool& debug);
    ~SystemV1();
private:
    class Impl;

    SystemV1()                = delete;
    SystemV1(const SystemV1&) = delete;
    std::unique_ptr<Impl> instance_;

    static std::unique_ptr<SystemV1> v1;
};
}
