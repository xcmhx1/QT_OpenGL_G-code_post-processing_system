#pragma once

#include "core/diagnostics/OperationContext.h"
#include "core/diagnostics/OperationResult.h"
#include "core/geometry/GeometryCompiler.h"
#include "core/nc/NcProgram.h"

#include <cstdint>
#include <optional>
#include <vector>

namespace cadcam::nc
{
    struct PlanarNcEntityInput
    {
        geometry::SourceEntity sourceEntity;
        NcEntityMetadata metadata;
        bool reverse = false;
        std::optional<double> startParameter;
    };

    struct PlanarNcBuildPolicy
    {
        PlanarNcBuildPolicy()
        {
            samplingPolicy.chordTolerance = 0.0;
            samplingPolicy.fullTurnSegments = 128;
            samplingPolicy.minimumSegments = 16;
        }

        geometry::SamplingPolicy samplingPolicy;
        double principalPlaneTolerance = 1.0e-6;
        bool preserveCurrentOutputQuantization = true;
    };

    class PlanarNcProgramBuilder
    {
    public:
        static OperationResult<NcProgram> build
        (
            std::uint64_t contentRevision,
            const std::vector<PlanarNcEntityInput>& entities,
            const PlanarNcBuildPolicy& policy,
            const OperationContext& context,
            std::uint64_t processStateRevision = 1
        );
    };
}
