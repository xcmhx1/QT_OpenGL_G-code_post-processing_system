// 声明 CadViewerUtils 模块，对外暴露当前组件的核心类型、接口和协作边界。
// Viewer 辅助模块，整理视图层复用的工具函数和辅助计算逻辑。
#pragma once

#include <QMatrix4x4>
#include <QPointF>
#include <QVector3D>

#include "CadRenderTypes.h"

class CadItem;

namespace CadViewerUtils
{
    EntityId toEntityId(const CadItem* entity);
    QPointF projectToScreen
    (
        const QVector3D& worldPos,
        const QMatrix4x4& viewProjection,
        int viewportWidth,
        int viewportHeight
    );
    float distanceToSegmentSquared(const QPointF& point, const QPointF& start, const QPointF& end);
    GLenum primitiveTypeForEntity(const CadItem* entity);
    QVector3D flattenedToGroundPlane(const QVector3D& point);
}
