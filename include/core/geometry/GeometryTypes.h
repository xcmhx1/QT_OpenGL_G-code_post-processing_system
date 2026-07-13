#pragma once

#include <cstdint>
#include <variant>

namespace cadcam::geometry
{
    using EntityId = std::uint64_t;

    struct Vector3d
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    enum class SourceGeometryKind
    {
        Line,
        Arc,
        Circle,
        Ellipse,
        Polyline,
        Spline,
        Point,
        Unknown
    };

    struct LineGeometry
    {
        Vector3d start;
        Vector3d end;
    };

    struct CircleGeometry
    {
        Vector3d center;
        Vector3d axisU;
        Vector3d axisV;
        double radius = 0.0;
    };

    struct ArcGeometry
    {
        Vector3d center;
        Vector3d axisU;
        Vector3d axisV;
        double radius = 0.0;
        double startParameter = 0.0;
        double endParameter = 0.0;
    };

    struct EllipseGeometry
    {
        Vector3d center;
        Vector3d majorAxis;
        Vector3d minorAxis;
        double startParameter = 0.0;
        double endParameter = 0.0;
        bool fullEllipse = false;
    };

    using GeometryVariant = std::variant
    <
        LineGeometry,
        CircleGeometry,
        ArcGeometry,
        EllipseGeometry
    >;

    struct SourceEntity
    {
        EntityId id = 0;
        SourceGeometryKind kind = SourceGeometryKind::Unknown;
        GeometryVariant geometry = LineGeometry{};
    };

    const char* sourceGeometryKindName(SourceGeometryKind kind);
}
