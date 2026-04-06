#include "pch.h"

#include "CadViewTransform.h"

#include "CadViewerUtils.h"

#include <QVector4D>

#include <algorithm>

namespace CadViewTransform
{
    float aspectRatio(int viewportWidth, int viewportHeight)
    {
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
        const float x = (2.0f * static_cast<float>(screenPos.x()) / std::max(1, viewportWidth)) - 1.0f;
        const float y = 1.0f - (2.0f * static_cast<float>(screenPos.y()) / std::max(1, viewportHeight));

        const QMatrix4x4 inverse = camera.viewProjectionMatrix(aspectRatio(viewportWidth, viewportHeight)).inverted();
        const QVector4D world = inverse * QVector4D(x, y, depth, 1.0f);

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
        const QVector3D nearPoint = screenToWorld(camera, viewportWidth, viewportHeight, screenPos, -1.0f);
        const QVector3D farPoint = screenToWorld(camera, viewportWidth, viewportHeight, screenPos, 1.0f);
        const QVector3D rayDirection = farPoint - nearPoint;

        if (!qFuzzyIsNull(rayDirection.z()))
        {
            const float t = -nearPoint.z() / rayDirection.z();
            return nearPoint + rayDirection * t;
        }

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
        const QVector3D nearPoint = screenToWorld(camera, viewportWidth, viewportHeight, screenPos, -1.0f);
        const QVector3D farPoint = screenToWorld(camera, viewportWidth, viewportHeight, screenPos, 1.0f);
        const QVector3D rayDirection = farPoint - nearPoint;

        if (!qFuzzyIsNull(rayDirection.z()))
        {
            const float t = (planeZ - nearPoint.z()) / rayDirection.z();
            return nearPoint + rayDirection * t;
        }

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
        return camera.viewHeight / static_cast<float>(std::max(1, viewportHeight));
    }
}
