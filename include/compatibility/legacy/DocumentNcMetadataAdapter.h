#pragma once

#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/nc/NcProgram.h"
#include "core/planning/ProcessPlan.h"

class CadDocument;

class DocumentNcMetadataAdapter
{
public:
    static OperationResult<std::vector<cadcam::nc::NcEntityMetadata>> capture
    (
        CadDocument& document,
        const cadcam::planning::ProcessPlan& plan,
        const OperationContext& context
    );
};
