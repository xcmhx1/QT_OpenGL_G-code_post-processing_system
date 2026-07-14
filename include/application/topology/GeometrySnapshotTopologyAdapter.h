#pragma once

#include "application/geometry/GeometrySnapshot.h"
#include "core/topology/PathTopology.h"

class GeometrySnapshotTopologyAdapter
{
public:
    OperationResult<cadcam::topology::TopologyInput> convert
    (
        const GeometrySnapshot& snapshot,
        const std::vector<cadcam::geometry::EntityId>& subset,
        const cadcam::topology::PathTopologyTolerance& tolerance,
        const OperationContext& context
    ) const;
};
