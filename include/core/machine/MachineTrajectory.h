#pragma once

#include "core/geometry/Path3D.h"
#include "core/machining/TubeSection.h"
#include "core/planning/ProcessPlan.h"

#include <cstdint>
#include <optional>
#include <vector>

namespace cadcam::machine
{
    enum class MachineMoveKind { Rapid, Cutting, CuttingConnection, Overcut };
    enum class RotarySurfaceRegion { Top, Right, Bottom, Left, Corner, Radial, Unknown };

    struct MachinePose4D
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double aDegrees = 0.0;
    };

    struct MachineMove
    {
        MachineMoveKind kind = MachineMoveKind::Rapid;
        MachinePose4D target;
        geometry::EntityId entityId = 0;
        int processGroupId = -1;
    };

    struct EntityTrajectory
    {
        geometry::EntityId entityId = 0;
        geometry::SourceGeometryKind sourceKind = geometry::SourceGeometryKind::Unknown;
        std::size_t sourceIndex = 0;
        int processOrder = -1;
        int processGroupId = -1;
        bool closed = false;
        bool continuousFromPrevious = false;
        bool firstInGroup = false;
        bool lastInGroup = false;
        std::vector<geometry::Vector3d> sourcePath;
        std::vector<MachineMove> approachMoves;
        std::vector<MachineMove> cuttingMoves;
        std::vector<MachineMove> overcutMoves;
    };

    struct RotarySurfaceSummary
    {
        geometry::EntityId entityId = 0;
        int processGroupId = -1;
        geometry::SourceGeometryKind sourceKind = geometry::SourceGeometryKind::Unknown;
        std::size_t pointCount = 0;
        double ySpan = 0.0;
        double zSpan = 0.0;
        RotarySurfaceRegion classification = RotarySurfaceRegion::Unknown;
        double surfaceTolerance = 0.0;
        double rawAStart = 0.0;
        double rawAEnd = 0.0;
        double alignedAStart = 0.0;
        double alignedAEnd = 0.0;
    };

    struct RotaryTrajectoryContext
    {
        double rotaryAxisY = 0.0;
        double rotaryAxisZ = 0.0;
        double tubeCenterY = 0.0;
        double tubeCenterZ = 0.0;
        double maximumCollisionRadius = 0.0;
        double safeMachineZ = 0.0;
        bool hasSectionBounds = false;
        double sectionMinimumY = 0.0;
        double sectionMaximumY = 0.0;
        double sectionMinimumZ = 0.0;
        double sectionMaximumZ = 0.0;
    };

    struct MachineTrajectory
    {
        std::uint64_t contentRevision = 0;
        std::uint64_t processStateRevision = 0;
        RotaryTrajectoryContext rotaryContext;
        std::vector<EntityTrajectory> entities;
        std::vector<RotarySurfaceSummary> surfaceSummaries;
    };

    struct TrajectoryEntityInput
    {
        geometry::EntityId entityId = 0;
        std::size_t sourceIndex = 0;
        geometry::SourceGeometryKind sourceKind = geometry::SourceGeometryKind::Unknown;
        int processOrder = -1;
        int processGroupId = -1;
        bool closed = false;
        bool firstInGroup = false;
        bool lastInGroup = false;
        geometry::Path3D path;
    };

    struct RotaryMachinePolicy
    {
        double rotaryAxisY = 0.0;
        double rotaryAxisZ = 0.0;
        double tubeCenterY = 0.0;
        double tubeCenterZ = 0.0;
        bool invertAAxisDirection = false;
        double aAxisOffsetDegrees = 0.0;
        bool keepContinuousAngle = true;
        bool useInitialMachinePoint = false;
        MachinePose4D initialMachinePoint;
        bool useSafeZBeforeRapid = true;
        double safeRadialClearance = 5.0;
        double machiningPlaneZOffset = 0.0;
        double overcutDistance = 2.0;
        double continuousConnectionTolerance = 1.0;
        double numericalEpsilon = 1.0e-5;
        double surfaceClassificationTolerance = 1.0e-5;
    };

    struct RotaryTrajectoryInput
    {
        std::uint64_t contentRevision = 0;
        std::uint64_t processStateRevision = 0;
        std::vector<TrajectoryEntityInput> entities;
        std::vector<planning::ProcessGroup> processGroups;
        std::optional<machining::TubeSectionModel> tubeSection;
    };
}
