#pragma once

#include <QPoint>
#include <QVector3D>

#include "CadCamera.h"

namespace CadViewTransform
{
    float aspectRatio(int viewportWidth, int viewportHeight);
    QVector3D screenToWorld
    (
        const OrbitalCamera& camera,
        int viewportWidth,
        int viewportHeight,
        const QPoint& screenPos,
        float depth = 0.0f
    );
    QVector3D screenToGroundPlane
    (
        const OrbitalCamera& camera,
        int viewportWidth,
        int viewportHeight,
        const QPoint& screenPos
    );
    QVector3D screenToPlane
    (
        const OrbitalCamera& camera,
        int viewportWidth,
        int viewportHeight,
        const QPoint& screenPos,
        float planeZ
    );
    QPoint worldToScreen
    (
        const OrbitalCamera& camera,
        int viewportWidth,
        int viewportHeight,
        const QVector3D& worldPos
    );
    float pixelToWorldScale(const OrbitalCamera& camera, int viewportHeight);
}
