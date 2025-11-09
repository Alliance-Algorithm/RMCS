
#include "core/system_factory.hpp"
#include "../v1/auto_aim_system_v1.hpp"
#include "enum/system_version.hpp"
#include "tongji/auto_aim_system.hpp"
#include <format>

// 构建系统，Version 为特定版本
void world_exe::core::SystemFactory::Build(const enumeration::SystemVersion& version) {
    switch (version) {
    case enumeration::SystemVersion::V1:
        world_exe::v1::SystemV1::build(false); // 构建正常运行的V1版本
        break;
    case world_exe::enumeration::SystemVersion::V1Debug:
        world_exe::v1::SystemV1::build(true); // 构建调试运行的V1版本
        break;
    case world_exe::enumeration::SystemVersion::V2:
        world_exe::tongji::AutoAimSystem::build(false); // 构建调试运行的V1版本
        break;
    case world_exe::enumeration::SystemVersion::V2Debug:
        world_exe::tongji::AutoAimSystem::build(true); // 构建调试运行的V2版本
        break;
    default:
#if __cplusplus >= 202002L
        throw std::runtime_error(std::format("Target version 0x{:x} is not impleme \n Factory "
                                             "core/system_factory.cpp : Build(SystemVersion) ",
            (int)version));
#else
        throw std::runtime_error("Target version is not impleme \n Factory "
                                 "core/system_factory.cpp : Build(SystemVersion) ");

#endif
        break;
    }
}