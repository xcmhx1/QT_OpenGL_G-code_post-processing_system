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
