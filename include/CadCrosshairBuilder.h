// 声明 CadCrosshairBuilder 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 十字光标构建模块，负责生成视图中的辅助准星和拾取框预览图元。
#pragma once

#include <vector>

#include <QPoint>

#include "CadCamera.h"
#include "CadRenderTypes.h"

class CadCrosshairBuilder
{
public:
    static std::vector<TransientPrimitive> buildCrosshairPrimitives
    (
        const OrbitalCamera& camera,
        int viewportWidth,
        int viewportHeight,
        int widgetWidth,
        int widgetHeight,
        const QPoint& cursorScreenPos,
        bool visible,
        bool orbiting,
        float planeZ,
        float boxHalfSizeWorld,
        float crosshairHalfLengthWorld
    );
};
