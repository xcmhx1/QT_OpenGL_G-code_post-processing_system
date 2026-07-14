#pragma once

#include "application/geometry/GeometrySnapshot.h"
#include "application/tasks/TaskContext.h"

enum class GeometryExecutionMode
{
    Serial,
    Parallel
};

class GeometrySnapshotCompiler
{
public:
    OperationResult<GeometrySnapshot> compile
    (
        const GeometrySourceSnapshot& source,
        const cadcam::geometry::SamplingPolicy& policy,
        GeometryExecutionMode mode,
        const TaskContext& taskContext
    ) const;
};
