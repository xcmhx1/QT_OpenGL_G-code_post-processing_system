#pragma once

#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/planning/ProcessPlan.h"

namespace cadcam::planning
{
    struct PlanarPlanningEntity
    {
        geometry::EntityId entityId = 0;
        std::size_t sourceIndex = 0;
        geometry::SourceGeometryKind sourceKind = geometry::SourceGeometryKind::Unknown;
        geometry::SourceEntity sourceEntity;
        geometry::Path3D path;
        bool visible = true;
        bool processEnabled = true;
        bool excludedAsInternalGeometry = false;
        process::DirectionPreference directionPreference = process::DirectionPreference::Auto;
        std::optional<double> startParameter;
    };

    struct PlanarProcessPlanningInput
    {
        std::uint64_t contentRevision = 0;
        std::uint64_t processStateRevision = 0;
        std::vector<PlanarPlanningEntity> entities;
    };

    struct PlanarProcessPlanningPolicy
    {
        bool allowReverse = true;
        bool preserveUserDirection = true;
        geometry::Vector3d initialPosition;
        bool hasInitialPosition = false;
        double numericalEpsilon = 1.0e-5;
    };

    class PlanarProcessPlanBuilder
    {
    public:
        static OperationResult<ProcessPlan> build
        (
            const PlanarProcessPlanningInput& input,
            const PlanarProcessPlanningPolicy& policy,
            const OperationContext& context
        );
    };
}
