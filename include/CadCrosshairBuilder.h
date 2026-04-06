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
