// 声明 CadEntityPicker 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 实体拾取模块，负责基于屏幕空间距离规则选择当前命中的图元。
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
