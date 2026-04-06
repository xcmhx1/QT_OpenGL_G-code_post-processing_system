// 声明 CadCameraUtils 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 相机辅助模块，整理相机相关的公共工具函数和复用计算流程。
#pragma once

#include <QVector3D>

#include "CadCamera.h"

namespace CadCameraUtils
{
    void orbitCameraAroundPivot
    (
        OrbitalCamera& camera,
        const QVector3D& pivot,
        float deltaAzimuth,
        float deltaElevation
    );
}
