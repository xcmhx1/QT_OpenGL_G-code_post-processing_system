#pragma once

#include "core/geometry/Path3D.h"

namespace cadcam::geometry
{
    class GeometryCompiler
    {
    public:
        OperationResult<Path3D> compile
        (
            const SourceEntity& source,
            const SamplingPolicy& policy,
            const PathCompileOptions& options,
            const OperationContext& context
        ) const;
    };
}
