#pragma once

#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/geometry/GeometryTypes.h"

namespace cadcam::geometry
{
    class NurbsCurveEvaluator
    {
    public:
        OperationResult<Vector3d> evaluate
        (
            const SplineGeometry& spline,
            double parameter,
            const OperationContext& context
        ) const;
    };
}
