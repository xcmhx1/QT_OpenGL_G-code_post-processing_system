// 实现 CadCrosshairBuilder 模块，对应头文件中声明的主要行为和协作流程。
// 十字光标构建模块，负责生成视图中的辅助准星和拾取框预览图元。
#include "pch.h"

#include "CadCrosshairBuilder.h"

#include "CadInteractionConstants.h"
#include "CadViewTransform.h"

#include <algorithm>

namespace
{
    const QVector3D kCrosshairColor(0.38f, 0.88f, 1.0f);
}

std::vector<TransientPrimitive> CadCrosshairBuilder::buildCrosshairPrimitives
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
)
{
    std::vector<TransientPrimitive> primitives;

    if (!visible || widgetWidth <= 0 || widgetHeight <= 0 || orbiting)
    {
        return primitives;
    }

    const QPoint cursorPoint
    (
        std::clamp(cursorScreenPos.x(), 0, std::max(0, widgetWidth - 1)),
        std::clamp(cursorScreenPos.y(), 0, std::max(0, widgetHeight - 1))
    );
    const QVector3D worldCenter = CadViewTransform::screenToPlane(camera, viewportWidth, viewportHeight, cursorPoint, planeZ);

    TransientPrimitive horizontalLine;
    horizontalLine.primitiveType = GL_LINES;
    horizontalLine.color = kCrosshairColor;
    horizontalLine.vertices =
    {
        worldCenter + QVector3D(-crosshairHalfLengthWorld, 0.0f, 0.0f),
        worldCenter + QVector3D(crosshairHalfLengthWorld, 0.0f, 0.0f)
    };
    primitives.push_back(std::move(horizontalLine));

    TransientPrimitive verticalLine;
    verticalLine.primitiveType = GL_LINES;
    verticalLine.color = kCrosshairColor;
    verticalLine.vertices =
    {
        worldCenter + QVector3D(0.0f, -crosshairHalfLengthWorld, 0.0f),
        worldCenter + QVector3D(0.0f, crosshairHalfLengthWorld, 0.0f)
    };
    primitives.push_back(std::move(verticalLine));

    TransientPrimitive pickBox;
    pickBox.primitiveType = GL_LINE_STRIP;
    pickBox.color = kCrosshairColor;
    pickBox.vertices =
    {
        worldCenter + QVector3D(-boxHalfSizeWorld, -boxHalfSizeWorld, 0.0f),
        worldCenter + QVector3D(boxHalfSizeWorld, -boxHalfSizeWorld, 0.0f),
        worldCenter + QVector3D(boxHalfSizeWorld, boxHalfSizeWorld, 0.0f),
        worldCenter + QVector3D(-boxHalfSizeWorld, boxHalfSizeWorld, 0.0f),
        worldCenter + QVector3D(-boxHalfSizeWorld, -boxHalfSizeWorld, 0.0f)
    };
    primitives.push_back(std::move(pickBox));

    return primitives;
}
