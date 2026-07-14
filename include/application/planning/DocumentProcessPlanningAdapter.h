#pragma once

#include "application/process/DocumentProcessState.h"
#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/planning/PlanarProcessPlanBuilder.h"

class CadDocument;

class DocumentProcessPlanningAdapter
{
public:
    OperationResult<cadcam::planning::PlanarProcessPlanningInput> capturePlanar
    (
        CadDocument& document,
        const cadcam::process::DocumentProcessState& processState,
        const OperationContext& context
    ) const;

    OperationResult<cadcam::planning::ProcessPlanningInput> captureRotary
    (
        CadDocument& document,
        const cadcam::process::DocumentProcessState& processState,
        const std::optional<cadcam::machining::TubeSectionModel>& tubeSection,
        double connectionTolerance,
        cadcam::topology::PathTopology& topologyStorage,
        const OperationContext& context
    ) const;
};
