// 实现 CadViewTransform 模块，对应头文件中声明的主要行为和协作流程。
// 视图变换模块，负责屏幕坐标、世界坐标与相机参数之间的转换。
#include "pch.h"

#include "CadViewTransform.h"

#include "CadViewerUtils.h"

#include <QVector4D>

#include <algorithm>

namespace CadViewTransform
{
    float aspectRatio(int viewportWidth, int viewportHeight)
    {
        // 始终对高度做最小值保护，避免除零。
        return static_cast<float>(viewportWidth) / static_cast<float>(std::max(1, viewportHeight));
    }

    QVector3D screenToWorld
    (
        const OrbitalCamera& camera,
        int viewportWidth,
        int viewportHeight,
        const QPoint& screenPos,
        float depth
    )
    {
        // 先把屏幕像素坐标映射到 NDC 区间 [-1, 1]。
        const float x = (2.0f * static_cast<float>(screenPos.x()) / std::max(1, viewportWidth)) - 1.0f;
        const float y = 1.0f - (2.0f * static_cast<float>(screenPos.y()) / std::max(1, viewportHeight));

        // 使用 VP 矩阵逆矩阵把 NDC 反投影回世界空间。
        const QMatrix4x4 inverse = camera.viewProjectionMatrix(aspectRatio(viewportWidth, viewportHeight)).inverted();
        const QVector4D world = inverse * QVector4D(x, y, depth, 1.0f);

        // 无法透视除法时返回零向量作为兜底。
        if (qFuzzyIsNull(world.w()))
        {
            return QVector3D();
        }

        return world.toVector3DAffine();
    }

    QVector3D screenToGroundPlane
    (
        const OrbitalCamera& camera,
        int viewportWidth,
        int viewportHeight,
        const QPoint& screenPos
    )
    {
        // 先求出鼠标射线穿过近远裁剪面的两个世界点。
        const QVector3D nearPoint = screenToWorld(camera, viewportWidth, viewportHeight, screenPos, -1.0f);
        const QVector3D farPoint = screenToWorld(camera, viewportWidth, viewportHeight, screenPos, 1.0f);
        const QVector3D rayDirection = farPoint - nearPoint;

        // 用射线与 Z=0 平面的交点作为最终二维工作平面坐标。
        if (!qFuzzyIsNull(rayDirection.z()))
        {
            const float t = -nearPoint.z() / rayDirection.z();
            return nearPoint + rayDirection * t;
        }

        // 当射线几乎平行于平面时，退化为直接压回地平面。
        return CadViewerUtils::flattenedToGroundPlane(nearPoint);
    }

    QVector3D screenToPlane
    (
        const OrbitalCamera& camera,
        int viewportWidth,
        int viewportHeight,
        const QPoint& screenPos,
        float planeZ
    )
    {
        // 与 screenToGroundPlane 相同，只是目标平面高度可配置。
        const QVector3D nearPoint = screenToWorld(camera, viewportWidth, viewportHeight, screenPos, -1.0f);
        const QVector3D farPoint = screenToWorld(camera, viewportWidth, viewportHeight, screenPos, 1.0f);
        const QVector3D rayDirection = farPoint - nearPoint;

        if (!qFuzzyIsNull(rayDirection.z()))
        {
            const float t = (planeZ - nearPoint.z()) / rayDirection.z();
            return nearPoint + rayDirection * t;
        }

        // 射线平行于平面时，保留 XY 并直接强制设定目标 Z。
        return QVector3D(nearPoint.x(), nearPoint.y(), planeZ);
    }

    QPoint worldToScreen
    (
        const OrbitalCamera& camera,
        int viewportWidth,
        int viewportHeight,
        const QVector3D& worldPos
    )
    {
        // 世界转屏幕统一复用 ViewerUtils 的投影逻辑，避免两套实现不一致。
        return CadViewerUtils::projectToScreen
        (
            worldPos,
            camera.viewProjectionMatrix(aspectRatio(viewportWidth, viewportHeight)),
            viewportWidth,
            viewportHeight
        ).toPoint();
    }

    float pixelToWorldScale(const OrbitalCamera& camera, int viewportHeight)
    {
        // 当前相机以 viewHeight 描述可见范围，除以像素高度即可得到每像素世界尺度。
        return camera.viewHeight / static_cast<float>(std::max(1, viewportHeight));
    }
}
