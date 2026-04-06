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
