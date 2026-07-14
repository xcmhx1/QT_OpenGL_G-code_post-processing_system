#pragma once

#include "core/geometry/GeometryCompiler.h"

class DRW_Spline;

class SplineProductionPathProvider
{
public:
    static OperationResult<cadcam::geometry::Path3D> build
    (
        cadcam::geometry::EntityId entityId,
        const DRW_Spline& spline,
        const cadcam::geometry::SamplingPolicy& policy,
        const cadcam::geometry::PathCompileOptions& options,
        const OperationContext& context
    );
};
