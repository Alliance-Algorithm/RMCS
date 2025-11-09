#include <gtest/gtest.h>
#include <interfaces/pnp_solver.hpp>
#include "mocks/MockArmor2D.hpp"
#include "interfaces/pnp_solver.hpp"

using world_exe::interfaces::IPnpSolver;
using world_exe::tests::mock::MockArmorInImage;

class PnpsolverTest : public ::testing::TestWithParam<IPnpSolver*> {
public:
    PnpsolverTest() : pnp_solver_test(GetParam()) {}

    void RunableTest()
    {
        auto mockarmor = MockArmorInImage::createMockArmorInImage();
        pnp_solver_test->SolvePnp(mockarmor);
    }
    void RunTest() {
        RunableTest();
    }
private:
    IPnpSolver* pnp_solver_test;
};