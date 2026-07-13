#pragma once

#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/geometry/GeometryTypes.h"

#include <optional>
#include <vector>

namespace cadcam::geometry
{
    struct PathVertex3D
    {
        Vector3d position;
        double sourceParameter = 0.0;
    };

    struct Path3D
    {
        EntityId sourceEntityId = 0;
        SourceGeometryKind sourceKind = SourceGeometryKind::Unknown;
        std::vector<PathVertex3D> vertices;
        bool closed = false;
        double samplingTolerance = 0.0;
    };

    struct SamplingPolicy
    {
        double chordTolerance = 0.01;
        double maximumSegmentLength = 0.0;
        double maximumAngularStep = 0.0;
        int minimumSegments = 1;
        int fullTurnSegments = 128;
        int maximumSegments = 65536;
        bool singlePrecisionEvaluation = false;
    };

    struct PathCompileOptions
    {
        bool reverse = false;
        std::optional<double> startParameter;
    };

    OperationReport validatePath3D
    (
        const Path3D& path,
        const OperationContext& context
    );
}
