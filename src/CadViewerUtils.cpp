// 实现 CadViewerUtils 模块，对应头文件中声明的主要行为和协作流程。
// Viewer 辅助模块，整理视图层复用的工具函数和辅助计算逻辑。
#include "pch.h"

#include "CadViewerUtils.h"

#include "CadItem.h"

#include <QVector4D>

#include <algorithm>

namespace CadViewerUtils
{
    EntityId toEntityId(const CadItem* entity)
    {
        return static_cast<EntityId>(reinterpret_cast<quintptr>(entity));
    }

    QPointF projectToScreen
    (
        const QVector3D& worldPos,
        const QMatrix4x4& viewProjection,
        int viewportWidth,
        int viewportHeight
    )
    {
        const QVector4D clip = viewProjection * QVector4D(worldPos, 1.0f);

        if (qFuzzyIsNull(clip.w()))
        {
            return QPointF();
        }

        const QVector3D ndc = clip.toVector3DAffine();

        return QPointF
        (
            (ndc.x() + 1.0f) * 0.5f * viewportWidth,
            (1.0f - ndc.y()) * 0.5f * viewportHeight
        );
    }

    float distanceToSegmentSquared(const QPointF& point, const QPointF& start, const QPointF& end)
    {
        const QPointF segment = end - start;
        const double lengthSquared = segment.x() * segment.x() + segment.y() * segment.y();

        if (lengthSquared <= 1.0e-12)
        {
            const QPointF delta = point - start;
            return static_cast<float>(delta.x() * delta.x() + delta.y() * delta.y());
        }

        const QPointF fromStart = point - start;
        const double t = std::clamp
        (
            (fromStart.x() * segment.x() + fromStart.y() * segment.y()) / lengthSquared,
            0.0,
            1.0
        );

        const QPointF projection = start + segment * t;
        const QPointF delta = point - projection;
        return static_cast<float>(delta.x() * delta.x() + delta.y() * delta.y());
    }

    GLenum primitiveTypeForEntity(const CadItem* entity)
    {
        if (entity == nullptr)
        {
            return GL_LINE_STRIP;
        }

        switch (entity->m_type)
        {
        case DRW::ETYPE::POINT:
            return GL_POINTS;
        case DRW::ETYPE::LINE:
            return GL_LINES;
        default:
            return GL_LINE_STRIP;
        }
    }

    QVector3D flattenedToGroundPlane(const QVector3D& point)
    {
        return QVector3D(point.x(), point.y(), 0.0f);
    }
}
