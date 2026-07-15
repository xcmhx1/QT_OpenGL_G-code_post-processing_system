#pragma once

#include "core/geometry/GeometryTypes.h"

#include <vector>

namespace cadcam::geometry
{
    struct LocalFrame2d
    {
        Vector2d origin;
        Vector2d toLocal(const Vector2d& world) const;
        Vector2d toWorld(const Vector2d& local) const;
    };

    struct LocalFrame3d
    {
        Vector3d origin;
        Vector3d toLocal(const Vector3d& world) const;
        Vector3d toWorld(const Vector3d& local) const;
    };

    struct PlaneFrame3d
    {
        Vector3d origin;
        Vector3d axisU;
        Vector3d axisV;
        Vector3d normal;
        Vector3d toLocal(const Vector3d& world) const;
        Vector3d toWorld(const Vector3d& local) const;
    };

    Vector2d stableBoundsCenter(const std::vector<Vector2d>& points);
    Vector3d stableBoundsCenter(const std::vector<Vector3d>& points);
}
