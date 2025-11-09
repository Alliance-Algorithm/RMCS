#include "v1/pnpsolver/armor_pnp_solver.hpp"
#include <core/system_factory.hpp>
#include <enum/system_version.hpp>
#include <filesystem>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <parameters/params_system_v1.hpp>
#include <parameters/rm_parameters.hpp>
#include <v1/identifier/identifier.hpp>
#include "Pnpsolver.cpp"
#include "sync_test.cpp"
#include "triple_buffer.cpp"

using namespace world_exe::v1::pnpsolver;
using world_exe::parameters::Robomaster;

using ::testing::Return;
using ::testing::_;


TEST_P(PnpsolverTest,AbilityTest)
{
    ArmorIPPEPnPSolver pnp_solver_test_v1(Robomaster::NormalArmorObjectPointsOpencv,Robomaster::LargeArmorObjectPointsOpencv);
    RunTest();
}

TEST(V1Test,SyncTest)
{
    world_exe::tests::sync::sync_test_main();
}


TEST(V1Build, SystemBuildTest){

    std::cout << "Current path: " << std::filesystem::current_path() << std::endl;
    world_exe::parameters::ParamsForSystemV1::set_szu_model_path("../../models/szu_identify_model.onnx");
    world_exe::core::SystemFactory::Build(world_exe::enumeration::SystemVersion::V1);
    world_exe::core::SystemFactory::Build(world_exe::enumeration::SystemVersion::V1Debug);
}

INSTANTIATE_TEST_SUITE_P(PnpsolverTest, PnpsolverTest, ::testing::Values(
    new ArmorIPPEPnPSolver(Robomaster::NormalArmorObjectPointsOpencv,Robomaster::LargeArmorObjectPointsOpencv)
    ));

    // INSTANTIATE__SUITE_P()
    // 测试套件
    // 测试类名
    // 测试类的参数传入->GetParam()
int main(int argc, char **argv) {
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}