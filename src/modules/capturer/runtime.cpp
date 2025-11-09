#include "runtime.hpp"
#include "modules/capturer/hikcamera.hpp"

namespace rmcs::cap {

struct Runtime::Impl { };

Runtime::Runtime() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Runtime::~Runtime() noexcept = default;

}
