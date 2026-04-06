#pragma once

#include <memory>
#include <vector>

#include <QMatrix4x4>
#include <QPoint>

#include "CadRenderTypes.h"

class CadItem;

namespace CadEntityPicker
{
    EntityId pickEntity
    (
        const std::vector<std::unique_ptr<CadItem>>& entities,
        const QMatrix4x4& viewProjection,
        int viewportWidth,
        int viewportHeight,
        const QPoint& screenPos,
        float pickThresholdPixels
    );
}
