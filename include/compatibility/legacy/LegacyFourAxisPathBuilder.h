#pragma once

#include "CadItem.h"
#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"

struct LegacyFourAxisPathOptions
{
    double axisY = 0.0;
    double axisZ = 0.0;
    double judgeCenterY = 0.0;
    double judgeCenterZ = 0.0;
    bool invertAAxisDirection = false;
    double aAxisOffsetDegrees = 0.0;
    bool keepContinuousAngle = true;
};

class LegacyFourAxisPathBuilder
{
public:
    static OperationResult<std::vector<ControlPoint4Axis>> build
    (
        const std::vector<RawPathPoint3D>& rawPath,
        const LegacyFourAxisPathOptions& options,
        cadcam::geometry::EntityId entityId,
        const OperationContext& context
    );
};
