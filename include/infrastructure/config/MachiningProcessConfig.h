#pragma once

#include "infrastructure/config/GProfile.h"

// 加工工艺配置的单一值对象。
// 由 GProfile（含长期参数覆盖）在应用层折叠形成，供轨迹与 NC 服务统一消费，
// 避免各服务逐字段复制传递 GProfileRotaryAxisConfig / GProfileToolTransferConfig。
struct MachiningProcessConfig
{
    // 旋转轴
    double rotaryCenterY = 0.0;
    double rotaryCenterZ = 0.0;
    double aAxisOffsetDegrees = 0.0;
    bool invertAAxisDirection = false;
    bool keepContinuousAngle = true;
    bool useSafeZBeforeRapid = true;
    bool useInitialMachinePoint = false;
    double initialMachineX = 0.0;
    double initialMachineY = 0.0;
    double initialMachineZ = 0.0;

    // 安全转移
    double rotationSafetyClearance = 5.0;
    double sameZoneTransferClearance = 0.0;
    bool coordinatedTransferEnabled = true;

    // 加工工艺
    double machiningPlaneZOffset = 0.0;
    double overcutDistance = 2.0;

    // 单元成员连续连接容差：必须与规划使用的连接容差一致。
    double continuousConnectionTolerance = 1.0;

    static MachiningProcessConfig fromProfile(const GProfile& profile)
    {
        const GProfileRotaryAxisConfig& rotary = profile.rotaryAxisConfig();
        const GProfileToolTransferConfig& transfer = profile.toolTransferConfig();
        return fromConfigs(rotary, transfer);
    }

    static MachiningProcessConfig fromConfigs
    (
        const GProfileRotaryAxisConfig& rotary,
        const GProfileToolTransferConfig& transfer
    )
    {
        MachiningProcessConfig config;
        config.rotaryCenterY = rotary.centerY;
        config.rotaryCenterZ = rotary.centerZ;
        config.aAxisOffsetDegrees = rotary.aAxisOffsetDegrees;
        config.invertAAxisDirection = rotary.invertAAxisDirection;
        config.keepContinuousAngle = rotary.keepContinuousAngle;
        config.useSafeZBeforeRapid = rotary.useSafeZBeforeRapid;
        config.useInitialMachinePoint = rotary.useInitialMachinePoint;
        config.initialMachineX = rotary.initialMachineX;
        config.initialMachineY = rotary.initialMachineY;
        config.initialMachineZ = rotary.initialMachineZ;
        config.rotationSafetyClearance = transfer.rotationSafetyClearance;
        config.sameZoneTransferClearance = transfer.sameZoneTransferClearance;
        config.coordinatedTransferEnabled = transfer.coordinatedTransferEnabled;
        config.machiningPlaneZOffset = rotary.machiningPlaneZOffset;
        config.overcutDistance = rotary.overcutDistance;
        return config;
    }
};
