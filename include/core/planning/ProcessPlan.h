#pragma once

#include "core/geometry/Path3D.h"
#include "core/machining/TubeSection.h"
#include "core/topology/PathTopology.h"

#include <cstdint>
#include <optional>
#include <vector>

namespace cadcam::planning
{
    enum class ProcessPlanMode { Planar3Axis, Rotary4Axis };
    enum class ProcessOrderingStrategy { NearestNext, LazyRotation };
    enum class BoundaryRole { None, Break, Waste };
    enum class ProcessGroupKind { SingleEntity, ConnectedChain, ClosedLoop, BreakBoundary, WasteBoundary };
    enum class ProcessExclusionReason { Hidden, UserDisabled, InternalGeometry, WasteRegion, UnsupportedGeometry, InvalidPath };
    enum class BoundarySide { Left, OnBoundary, Right, Mixed, Indeterminate };

    struct ProcessPlanningPolicy
    {
        ProcessOrderingStrategy orderingStrategy = ProcessOrderingStrategy::NearestNext;
        double connectionTolerance = 1.0;
        bool allowReverse = true;
        bool preserveClosedLoopsAsAtomicGroups = true;
        geometry::Vector3d initialPosition{ 0.0, 0.0, 500.0 };
    };

    struct PlanningEntity
    {
        geometry::EntityId entityId = 0;
        std::size_t sourceIndex = 0;
        geometry::SourceGeometryKind sourceKind = geometry::SourceGeometryKind::Unknown;
        geometry::Path3D path;
        bool visible = true;
        bool processEnabled = true;
        bool excludedAsInternalGeometry = false;
        BoundaryRole boundaryRole = BoundaryRole::None;
        int boundaryPairId = -1;
        bool currentReverse = false;
        std::optional<double> currentStartParameter;
    };

    struct ProcessAssignment
    {
        geometry::EntityId entityId = 0;
        int processOrder = -1;
        int continuousGroupId = -1;
        bool reverse = false;
        std::optional<double> startParameter;
    };

    struct ProcessGroup
    {
        int groupId = -1;
        ProcessGroupKind kind = ProcessGroupKind::SingleEntity;
        bool closed = false;
        std::vector<geometry::EntityId> entityIds;
    };

    struct ProcessExclusion
    {
        geometry::EntityId entityId = 0;
        ProcessExclusionReason reason = ProcessExclusionReason::InvalidPath;
    };

    struct ProcessPrecedence
    {
        int predecessorGroupId = -1;
        int successorGroupId = -1;
        int boundaryPairId = -1;
    };

    struct ProcessPlan
    {
        std::uint64_t contentRevision = 0;
        ProcessPlanMode mode = ProcessPlanMode::Planar3Axis;
        ProcessOrderingStrategy orderingStrategy = ProcessOrderingStrategy::NearestNext;
        std::vector<ProcessAssignment> assignments;
        std::vector<ProcessGroup> groups;
        std::vector<ProcessExclusion> exclusions;
        std::vector<ProcessPrecedence> precedenceConstraints;
    };

    struct ProcessPlanningInput
    {
        std::uint64_t contentRevision = 0;
        std::vector<PlanningEntity> entities;
        topology::TopologyInput topologyInput;
        const topology::PathTopology* topology = nullptr;
        std::optional<machining::TubeSectionModel> tubeSection;
    };
}
