#pragma once

#include "core/geometry/Path3D.h"
#include "core/machining/TubeSection.h"
#include "core/machining/TubeSectionProjector.h"
#include "core/topology/PathTopology.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <optional>
#include <set>
#include <vector>

namespace cadcam::process
{
    enum class DirectionPreference { Auto, Forward, Reverse };
}

namespace cadcam::planning
{
    enum class ProcessPlanMode { Planar3Axis, Rotary4Axis };
    enum class ProcessOrderingStrategy { NearestNext, LazyRotation };
    enum class ProcessSortIntent { PreserveCurrentSequence, RebuildSequence };
    enum class PerimeterSweepDirection { Clockwise, CounterClockwise };
    enum class LongitudinalSweepDirection { PositiveX, NegativeX };
    enum class BoundaryRole { None, Break, Waste };
    enum class ProcessGroupKind { SingleEntity, ConnectedChain, ClosedLoop, BreakBoundary, WasteBoundary };
    enum class ProcessExclusionReason { Hidden, UserDisabled, InternalGeometry, WasteRegion, UnsupportedGeometry, InvalidPath };
    enum class BoundarySide { Left, OnBoundary, Right, Mixed, Indeterminate };

    struct Zone16SweepPolicy
    {
        machining::TubeZone16 initialZone = machining::TubeZone16::TopFace;
        PerimeterSweepDirection perimeterDirection =
            PerimeterSweepDirection::Clockwise;
        LongitudinalSweepDirection longitudinalDirection =
            LongitudinalSweepDirection::PositiveX;
    };

    struct ProcessPlanningPolicy
    {
        ProcessSortIntent sortIntent = ProcessSortIntent::RebuildSequence;
        ProcessOrderingStrategy orderingStrategy = ProcessOrderingStrategy::NearestNext;
        double connectionTolerance = 1.0;
        bool allowReverse = true;
        bool preserveClosedLoopsAsAtomicGroups = true;
        geometry::Vector3d initialPosition{ 0.0, 0.0, 500.0 };
        Zone16SweepPolicy zone16Sweep;
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
        process::DirectionPreference directionPreference = process::DirectionPreference::Auto;
        std::optional<double> startParameter;
    };

    struct ProcessUnitKey
    {
        std::vector<geometry::EntityId> memberEntityIds;

        bool operator==(const ProcessUnitKey& other) const
        {
            return memberEntityIds == other.memberEntityIds;
        }
    };

    struct ProcessUnit
    {
        ProcessUnitKey key;
        std::vector<geometry::EntityId> orderedMemberEntityIds;
        bool closed = false;
    };

    struct ProcessUnitSequence
    {
        std::vector<ProcessUnitKey> units;
        std::uint64_t revision = 1;
    };

    struct ProcessAssignment
    {
        geometry::EntityId entityId = 0;
        int processOrder = -1;
        int processUnitIndex = -1;
        int continuousGroupId = -1;
        bool reverse = false;
        std::optional<double> startParameter;
    };

    struct ProcessPathFragment
    {
        geometry::EntityId entityId = 0;
        int processUnitIndex = -1;
        int fragmentOrder = -1;
        double sourceParameterBegin = 0.0;
        double sourceParameterEnd = 0.0;
        bool reverse = false;
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
        std::uint64_t processStateRevision = 0;
        ProcessPlanMode mode = ProcessPlanMode::Planar3Axis;
        ProcessOrderingStrategy orderingStrategy = ProcessOrderingStrategy::NearestNext;
        std::vector<ProcessUnit> processUnits;
        ProcessUnitSequence processUnitSequence;
        std::vector<ProcessAssignment> assignments;
        std::vector<ProcessPathFragment> plannedFragments;
        std::vector<ProcessGroup> groups;
        std::vector<ProcessExclusion> exclusions;
        std::vector<ProcessPrecedence> precedenceConstraints;
    };

    inline bool validProcessUnitKey(const ProcessUnitKey& key)
    {
        return !key.memberEntityIds.empty()
            && key.memberEntityIds.front() != 0U
            && std::adjacent_find
                (key.memberEntityIds.begin(), key.memberEntityIds.end(), std::greater_equal<>())
                == key.memberEntityIds.end();
    }

    inline bool validateProcessUnitStructure(const ProcessPlan& plan)
    {
        if (plan.processUnitSequence.revision == 0U
            || plan.processUnits.size() != plan.processUnitSequence.units.size())
        {
            return false;
        }

        std::set<std::vector<geometry::EntityId>> unitKeys;
        std::size_t expectedAssignmentCount = 0U;
        for (std::size_t index = 0; index < plan.processUnits.size(); ++index)
        {
            const ProcessUnit& unit = plan.processUnits[index];
            if (!validProcessUnitKey(unit.key)
                || !(unit.key == plan.processUnitSequence.units[index])
                || unit.orderedMemberEntityIds.size() != unit.key.memberEntityIds.size()
                || !unitKeys.insert(unit.key.memberEntityIds).second)
            {
                return false;
            }

            std::vector<geometry::EntityId> orderedSet = unit.orderedMemberEntityIds;
            std::sort(orderedSet.begin(), orderedSet.end());
            if (orderedSet != unit.key.memberEntityIds
                || std::adjacent_find(orderedSet.begin(), orderedSet.end()) != orderedSet.end())
            {
                return false;
            }
            expectedAssignmentCount += unit.orderedMemberEntityIds.size();
        }

        if (expectedAssignmentCount != plan.assignments.size()) return false;

        std::vector<std::vector<geometry::EntityId>> assignedByUnit(plan.processUnits.size());
        std::vector<int> lastAssignmentByUnit(plan.processUnits.size(), -1);
        std::set<geometry::EntityId> assignedIds;
        for (std::size_t index = 0; index < plan.assignments.size(); ++index)
        {
            const ProcessAssignment& assignment = plan.assignments[index];
            if (assignment.entityId == 0U
                || assignment.processOrder != static_cast<int>(index)
                || assignment.processUnitIndex < 0
                || static_cast<std::size_t>(assignment.processUnitIndex) >= plan.processUnits.size()
                || !assignedIds.insert(assignment.entityId).second)
            {
                return false;
            }

            const std::size_t unitIndex = static_cast<std::size_t>(assignment.processUnitIndex);
            if (lastAssignmentByUnit[unitIndex] >= 0
                && lastAssignmentByUnit[unitIndex] + 1 != static_cast<int>(index))
            {
                return false;
            }
            lastAssignmentByUnit[unitIndex] = static_cast<int>(index);
            assignedByUnit[unitIndex].push_back(assignment.entityId);
        }

        for (std::size_t index = 0; index < plan.processUnits.size(); ++index)
        {
            if (assignedByUnit[index] != plan.processUnits[index].orderedMemberEntityIds) return false;
        }

        std::vector<std::vector<const ProcessPathFragment*>> fragmentsByUnit
            (plan.processUnits.size());
        for (const ProcessPathFragment& fragment : plan.plannedFragments)
        {
            if (fragment.entityId == 0U
                || fragment.processUnitIndex < 0
                || static_cast<std::size_t>(fragment.processUnitIndex) >= plan.processUnits.size()
                || fragment.fragmentOrder < 0
                || !std::isfinite(fragment.sourceParameterBegin)
                || !std::isfinite(fragment.sourceParameterEnd)
                || fragment.sourceParameterBegin == fragment.sourceParameterEnd)
            {
                return false;
            }
            const ProcessUnit& unit =
                plan.processUnits[static_cast<std::size_t>(fragment.processUnitIndex)];
            if (!std::binary_search(unit.key.memberEntityIds.begin(),
                unit.key.memberEntityIds.end(), fragment.entityId))
            {
                return false;
            }
            fragmentsByUnit[static_cast<std::size_t>(fragment.processUnitIndex)]
                .push_back(&fragment);
        }
        for (std::size_t unitIndex = 0; unitIndex < fragmentsByUnit.size(); ++unitIndex)
        {
            auto& fragments = fragmentsByUnit[unitIndex];
            if (fragments.empty()) continue;
            const ProcessUnit& unit = plan.processUnits[unitIndex];
            const auto matchingBreak = std::find_if
            (
                plan.groups.cbegin(), plan.groups.cend(),
                [&unit](const ProcessGroup& group)
                {
                    if (group.kind != ProcessGroupKind::BreakBoundary)
                        return false;
                    std::vector<geometry::EntityId> groupIds = group.entityIds;
                    std::sort(groupIds.begin(), groupIds.end());
                    groupIds.erase(std::unique(groupIds.begin(), groupIds.end()),
                        groupIds.end());
                    return groupIds == unit.key.memberEntityIds;
                }
            );
            if (matchingBreak == plan.groups.cend()) return false;
            std::sort(fragments.begin(), fragments.end(),
                [](const ProcessPathFragment* left, const ProcessPathFragment* right)
                {
                    return left->fragmentOrder < right->fragmentOrder;
                });
            std::set<geometry::EntityId> fragmentedIds;
            for (std::size_t fragmentIndex = 0;
                fragmentIndex < fragments.size(); ++fragmentIndex)
            {
                if (fragments[fragmentIndex]->fragmentOrder
                    != static_cast<int>(fragmentIndex))
                {
                    return false;
                }
                fragmentedIds.insert(fragments[fragmentIndex]->entityId);
            }
            if (fragmentedIds.size()
                    != plan.processUnits[unitIndex].key.memberEntityIds.size()
                || !std::equal(fragmentedIds.begin(), fragmentedIds.end(),
                    plan.processUnits[unitIndex].key.memberEntityIds.begin()))
            {
                return false;
            }
        }
        return true;
    }

    struct ProcessPlanningInput
    {
        std::uint64_t contentRevision = 0;
        std::uint64_t processStateRevision = 0;
        std::vector<PlanningEntity> entities;
        topology::TopologyInput topologyInput;
        const topology::PathTopology* topology = nullptr;
        std::optional<machining::TubeSectionModel> tubeSection;
        std::optional<geometry::Vector2d> tubeSectionCenter;
    };
}
