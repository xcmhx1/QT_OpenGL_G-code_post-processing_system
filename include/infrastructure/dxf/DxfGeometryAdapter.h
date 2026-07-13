#pragma once

#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/geometry/GeometryTypes.h"

class DRW_Entity;

class DxfGeometryAdapter
{
public:
    static OperationResult<cadcam::geometry::SourceEntity> convert
    (
        cadcam::geometry::EntityId entityId,
        const DRW_Entity& entity,
        const OperationContext& context
    );
};
