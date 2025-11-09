#include "utility/node.hpp"
#include <memory>

namespace rmcs::kernel {

class AutoAim final : public util::Node {
public:
    explicit AutoAim() noexcept;
    ~AutoAim() noexcept override;

    AutoAim(const AutoAim&)            = delete;
    AutoAim& operator=(const AutoAim&) = delete;

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} // namespace rmcs
