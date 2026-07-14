#pragma once

#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/nc/PlanarNcProgramBuilder.h"

#include <cstdint>
#include <vector>

class CadDocument;

struct PlanarNcCapture
{
    std::uint64_t contentRevision = 0;
    std::vector<cadcam::nc::PlanarNcEntityInput> entities;
};

class DocumentPlanarNcInputAdapter
{
public:
    static OperationResult<PlanarNcCapture> capture
    (
        CadDocument& document,
        const OperationContext& context
    );
};
