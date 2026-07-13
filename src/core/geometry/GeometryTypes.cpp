#include "core/geometry/GeometryTypes.h"

namespace cadcam::geometry
{
    const char* sourceGeometryKindName(SourceGeometryKind kind)
    {
        switch (kind)
        {
        case SourceGeometryKind::Line: return "Line";
        case SourceGeometryKind::Arc: return "Arc";
        case SourceGeometryKind::Circle: return "Circle";
        case SourceGeometryKind::Ellipse: return "Ellipse";
        case SourceGeometryKind::Polyline: return "Polyline";
        case SourceGeometryKind::Spline: return "Spline";
        case SourceGeometryKind::Point: return "Point";
        case SourceGeometryKind::Unknown: return "Unknown";
        }

        return "Unknown";
    }
}
