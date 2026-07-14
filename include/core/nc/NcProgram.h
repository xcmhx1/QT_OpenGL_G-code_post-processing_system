#pragma once

#include "core/geometry/GeometryTypes.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace cadcam::nc
{
    enum class NcProgramMode { Planar3Axis, Rotary4Axis };
    enum class NcMotionKind { Rapid, Linear, CircularClockwise, CircularCounterclockwise };
    enum class NcSourceMoveKind { Rapid, Cutting, CuttingConnection, Overcut };
    enum class NcPlane { XY, ZX, YZ };

    struct NcAxisWords
    {
        std::optional<double> x;
        std::optional<double> y;
        std::optional<double> z;
        std::optional<double> a;
        std::optional<double> i;
        std::optional<double> j;
        std::optional<double> k;
        std::optional<double> r;
    };

    struct NcMotion
    {
        NcMotionKind kind = NcMotionKind::Linear;
        NcSourceMoveKind sourceKind = NcSourceMoveKind::Cutting;
        NcPlane plane = NcPlane::XY;
        NcAxisWords axes;
        geometry::EntityId entityId = 0;
        int processGroupId = -1;
    };

    struct NcEntityMetadata
    {
        geometry::EntityId entityId = 0;
        geometry::SourceGeometryKind sourceKind = geometry::SourceGeometryKind::Unknown;
        std::size_t sourceIndex = 0;
        int processOrder = -1;
        int processGroupId = -1;
        std::string entityTypeKey;
        std::string layerKey;
        std::string colorKey;
    };

    struct NcEntityBlock
    {
        NcEntityMetadata metadata;
        std::vector<NcMotion> motions;
    };

    struct NcComment
    {
        std::string text;
    };

    struct NcProgram
    {
        std::uint64_t contentRevision = 0;
        NcProgramMode mode = NcProgramMode::Rotary4Axis;
        std::vector<NcComment> leadingComments;
        std::vector<NcEntityBlock> entities;
    };
}
