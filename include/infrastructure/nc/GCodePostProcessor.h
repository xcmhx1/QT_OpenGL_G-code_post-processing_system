#pragma once

#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/nc/NcProgram.h"
#include "infrastructure/nc/GCodePostProcessorProfile.h"

#include <QString>

namespace cadcam::infrastructure::nc
{
    class GCodePostProcessor
    {
    public:
        static OperationResult<QString> render
        (
            const cadcam::nc::NcProgram& program,
            const GCodePostProcessorProfile& profile,
            const OperationContext& context
        );
    };
}
