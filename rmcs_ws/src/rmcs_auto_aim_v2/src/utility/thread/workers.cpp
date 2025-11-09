#include "workers.hpp"
#include <queue>

using namespace rmcs;

struct WorkersContext::Impl {
    std::queue<std::unique_ptr<InternalTask>> tasks;
};

WorkersContext::WorkersContext() noexcept
    : pimpl { std::make_unique<Impl>() } { }

WorkersContext::~WorkersContext() noexcept = default;
