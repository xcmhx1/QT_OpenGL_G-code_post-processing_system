#include "core/geometry/LocalCoordinateFrame.h"

#include <algorithm>

namespace cadcam::geometry
{
    namespace
    {
        double dot(const Vector3d& left, const Vector3d& right)
        {
            return left.x * right.x + left.y * right.y + left.z * right.z;
        }
    }

    Vector2d LocalFrame2d::toLocal(const Vector2d& world) const
    {
        return { world.x - origin.x, world.y - origin.y };
    }

    Vector2d LocalFrame2d::toWorld(const Vector2d& local) const
    {
        return { origin.x + local.x, origin.y + local.y };
    }

    Vector3d LocalFrame3d::toLocal(const Vector3d& world) const
    {
        return { world.x - origin.x, world.y - origin.y, world.z - origin.z };
    }

    Vector3d LocalFrame3d::toWorld(const Vector3d& local) const
    {
        return { origin.x + local.x, origin.y + local.y, origin.z + local.z };
    }

    Vector3d PlaneFrame3d::toLocal(const Vector3d& world) const
    {
        const Vector3d relative
        {
            world.x - origin.x,
            world.y - origin.y,
            world.z - origin.z
        };
        return { dot(relative, axisU), dot(relative, axisV), dot(relative, normal) };
    }

    Vector3d PlaneFrame3d::toWorld(const Vector3d& local) const
    {
        return
        {
            origin.x + axisU.x * local.x + axisV.x * local.y + normal.x * local.z,
            origin.y + axisU.y * local.x + axisV.y * local.y + normal.y * local.z,
            origin.z + axisU.z * local.x + axisV.z * local.y + normal.z * local.z
        };
    }

    Vector2d stableBoundsCenter(const std::vector<Vector2d>& points)
    {
        if (points.empty()) return {};
        double minimumX = points.front().x;
        double maximumX = points.front().x;
        double minimumY = points.front().y;
        double maximumY = points.front().y;
        for (const Vector2d& point : points)
        {
            minimumX = std::min(minimumX, point.x);
            maximumX = std::max(maximumX, point.x);
            minimumY = std::min(minimumY, point.y);
            maximumY = std::max(maximumY, point.y);
        }
        return
        {
            minimumX + (maximumX - minimumX) * 0.5,
            minimumY + (maximumY - minimumY) * 0.5
        };
    }

    Vector3d stableBoundsCenter(const std::vector<Vector3d>& points)
    {
        if (points.empty()) return {};
        double minimumX = points.front().x;
        double maximumX = points.front().x;
        double minimumY = points.front().y;
        double maximumY = points.front().y;
        double minimumZ = points.front().z;
        double maximumZ = points.front().z;
        for (const Vector3d& point : points)
        {
            minimumX = std::min(minimumX, point.x);
            maximumX = std::max(maximumX, point.x);
            minimumY = std::min(minimumY, point.y);
            maximumY = std::max(maximumY, point.y);
            minimumZ = std::min(minimumZ, point.z);
            maximumZ = std::max(maximumZ, point.z);
        }
        return
        {
            minimumX + (maximumX - minimumX) * 0.5,
            minimumY + (maximumY - minimumY) * 0.5,
            minimumZ + (maximumZ - minimumZ) * 0.5
        };
    }
}
