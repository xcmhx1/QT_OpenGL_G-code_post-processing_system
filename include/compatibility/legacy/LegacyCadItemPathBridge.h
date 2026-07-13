#pragma once

#include "CadItem.h"
#include "core/geometry/GeometryCompiler.h"

class LegacyCadItemPathBridge
{
public:
    static cadcam::geometry::SamplingPolicy legacySamplingPolicy(const CadItem& item);

    static OperationResult<cadcam::geometry::Path3D> compile
    (
        const CadItem& item,
        const cadcam::geometry::SamplingPolicy& policy,
        const cadcam::geometry::PathCompileOptions& options,
        const OperationContext& context
    );

    static void copyToLegacyRawPath
    (
        const cadcam::geometry::Path3D& path,
        std::vector<RawPathPoint3D>& destination
    );
};
