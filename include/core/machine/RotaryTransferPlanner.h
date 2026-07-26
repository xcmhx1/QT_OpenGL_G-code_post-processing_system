#pragma once

#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/geometry/GeometryTypes.h"
#include "core/machining/TubeSection.h"
#include "core/machining/TubeSectionProjector.h"

#include <optional>
#include <vector>

namespace cadcam::machine
{
    enum class TransferMotionKind
    {
        InitialApproach,
        SameZoneSurfaceTransfer,
        SameZoneClearanceTransfer,
        CrossZoneRotaryTransfer
    };

    enum class TransferMotionPhase
    {
        None,
        SurfaceTransfer,
        CoordinatedDeparture,
        SafeRotaryTransfer,
        CoordinatedApproach
    };

    struct MachinePose4D
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double aDegrees = 0.0;
    };

    struct ToolTransferPolicy
    {
        double rotationSafetyClearance = 5.0;
        double sameZoneTransferClearance = 0.0;
        bool coordinatedTransferEnabled = true;
    };

    struct RotaryTransferRequest
    {
        MachinePose4D previousCutEnd;
        MachinePose4D nextCutStart;
        geometry::Vector3d previousSourceEnd;
        geometry::Vector3d nextSourceStart;
        int previousProcessUnitIndex = -1;
        int nextProcessUnitIndex = -1;
        std::optional<machining::TubeZone16> previousOwnerZone;
        std::optional<machining::TubeZone16> nextOwnerZone;
        ToolTransferPolicy policy;
        const std::optional<machining::TubeSectionModel>* tubeSection = nullptr;
        double tubeCenterY = 0.0;
        double tubeCenterZ = 0.0;
        double rotationSafeMachineZ = 0.0;
        double numericalEpsilon = 1.0e-5;
    };

    struct TransferPreviewCost
    {
        double linearDistance = 0.0;
        double rotaryDegrees = 0.0;
        int rotaryDirectionChanges = 0;
    };

    struct RotaryTransferPreview
    {
        TransferMotionKind kind = TransferMotionKind::InitialApproach;
        std::vector<MachinePose4D> targets;
        std::vector<TransferMotionPhase> phases;
        MachinePose4D finalApproachOrigin;
        MachinePose4D cutStart;
        TransferPreviewCost cost;
    };

    class RotaryTransferPlanner
    {
    public:
        static OperationResult<RotaryTransferPreview> preview
        (
            const RotaryTransferRequest& request,
            const OperationContext& context
        );
    };
}
