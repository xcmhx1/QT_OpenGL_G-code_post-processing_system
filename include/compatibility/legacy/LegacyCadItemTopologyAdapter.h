#pragma once

#include "core/topology/PathTopology.h"

#include <QVector>

#include <functional>

class CadItem;

class LegacyCadItemTopologyAdapter
{
public:
    OperationResult<cadcam::topology::TopologyInput> convert
    (
        const QVector<CadItem*>& items,
        const cadcam::topology::PathTopologyTolerance& tolerance,
        const OperationContext& context,
        const std::function<void(double)>& pathRebuildObserver = {}
    ) const;
};
