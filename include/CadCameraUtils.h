// CadCameraUtils 头文件
// 声明 CadCameraUtils 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 相机辅助模块，整理相机相关的公共工具函数和复用计算流程。
#pragma once

#include <QVector3D>

#include "CadCamera.h"

namespace CadCameraUtils
{
    // 围绕指定枢轴点旋转轨道相机
    // @param camera 待修改的轨道相机
    // @param pivot 旋转中心点
    // @param deltaAzimuth 方位角增量
    // @param deltaElevation 俯仰角增量
    void orbitCameraAroundPivot
    (
        OrbitalCamera& camera,
        const QVector3D& pivot,
        float deltaAzimuth,
        float deltaElevation
    );
}
