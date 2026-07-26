#pragma once

#include "core/geometry/Path3D.h"
#include "core/machine/RotaryTransferPlanner.h"
#include "core/machining/TubeSection.h"
#include "core/planning/ProcessPlan.h"

#include <cstdint>
#include <optional>
#include <vector>

namespace cadcam::machine
{
    enum class MachineMoveKind { Rapid, Cutting, CuttingConnection, Overcut };
    enum class RotarySurfaceRegion { Top, Right, Bottom, Left, Corner, Radial, Unknown };
    struct MachineMove
    {
        MachineMoveKind kind = MachineMoveKind::Rapid;
        MachinePose4D target;
        TransferMotionKind transferKind = TransferMotionKind::InitialApproach;
        TransferMotionPhase transferPhase = TransferMotionPhase::None;
        geometry::EntityId entityId = 0;
        int processGroupId = -1;
    };

    struct EntityTrajectory
    {
        geometry::EntityId entityId = 0;
        geometry::SourceGeometryKind sourceKind = geometry::SourceGeometryKind::Unknown;
        std::size_t sourceIndex = 0;
        int processOrder = -1;
        int sourceProcessOrder = -1;
        int fragmentOrder = -1;
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

    struct TransferMotionSummary
    {
        int fromProcessUnit = -1;
        int toProcessUnit = -1;
        std::optional<machining::TubeZone16> fromOwnerZone;
        std::optional<machining::TubeZone16> toOwnerZone;
        TransferMotionKind kind = TransferMotionKind::InitialApproach;
        MachinePose4D previousCutEnd;
        MachinePose4D plannedPreviousCutEnd;
        MachinePose4D actualPreviousCutEnd;
        geometry::Vector3d plannedPreviousSourceEnd;
        geometry::Vector3d actualPreviousSourceEnd;
        MachinePose4D nextCutStart;
        MachinePose4D departureTarget;
        MachinePose4D rotaryTransferTarget;
        MachinePose4D approachTarget;
        MachinePose4D plannedFinalApproachOrigin;
        MachinePose4D actualFinalApproachOrigin;
        double poseDelta = 0.0;
        double sourceDelta = 0.0;
        double deltaA = 0.0;
        double rotationSafetyClearance = 0.0;
        double sameZoneTransferClearance = 0.0;
        double rotationSafeMachineZ = 0.0;
        bool coordinated = false;
        bool surfaceTransfer = false;
        bool hasPlannedPreview = false;
        bool previewMatched = false;
        int segmentCount = 0;
    };

    struct MachineTrajectory
    {
        std::uint64_t contentRevision = 0;
        std::uint64_t processStateRevision = 0;
        RotaryTrajectoryContext rotaryContext;
        std::vector<EntityTrajectory> entities;
        std::vector<RotarySurfaceSummary> surfaceSummaries;
        std::vector<TransferMotionSummary> transferSummaries;
    };

    struct TrajectoryEntityInput
    {
        geometry::EntityId entityId = 0;
        std::size_t sourceIndex = 0;
        geometry::SourceGeometryKind sourceKind = geometry::SourceGeometryKind::Unknown;
        int processOrder = -1;
        int sourceProcessOrder = -1;
        int fragmentOrder = -1;
        int processGroupId = -1;
        int processUnitIndex = -1;
        std::optional<machining::TubeZone16> ownerZone;
        bool closed = false;
        bool firstInGroup = false;
        bool lastInGroup = false;
        std::optional<planning::PlannedTransferSignature>
            plannedIncomingTransfer;
        geometry::Path3D path;
    };

    struct ToolClearancePolicy
    {
        double retractClearance = 5.0;
        double approachClearance = 0.0;
    };

    struct TransferClassificationInput
    {
        int previousProcessUnitIndex = -1;
        int nextProcessUnitIndex = -1;
        std::optional<machining::TubeZone16> previousOwnerZone;
        std::optional<machining::TubeZone16> nextOwnerZone;
        MachinePose4D previousCutEnd;
        MachinePose4D nextCutStart;
        double aAxisTolerance = 1.0e-5;
    };

    struct PlanarTrajectoryEntityInput
    {
        geometry::EntityId entityId = 0;
        int processGroupId = -1;
        int processUnitIndex = -1;
        geometry::Vector3d cutStart;
        geometry::Vector3d cutEnd;
    };

    struct PlanarEntityTrajectory
    {
        geometry::EntityId entityId = 0;
        std::vector<MachinePose4D> approachPoses;
    };

    struct PlanarTrajectory
    {
        std::vector<PlanarEntityTrajectory> entities;
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
        ToolTransferPolicy transfer;
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
