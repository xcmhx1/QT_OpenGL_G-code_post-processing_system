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
        // 直接以对象地址作为运行期实体 ID，避免额外维护独立编号表。
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
        // 先把世界坐标变到裁剪空间，再转换到 NDC。
        const QVector4D clip = viewProjection * QVector4D(worldPos, 1.0f);

        // w 为 0 说明无法进行透视除法，此时返回空点作为兜底。
        if (qFuzzyIsNull(clip.w()))
        {
            return QPointF();
        }

        const QVector3D ndc = clip.toVector3DAffine();

        // Qt 屏幕坐标系的 Y 轴向下，因此这里对 NDC 的 Y 做翻转。
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

        // 退化线段直接退化为“点到点距离”。
        if (lengthSquared <= 1.0e-12)
        {
            const QPointF delta = point - start;
            return static_cast<float>(delta.x() * delta.x() + delta.y() * delta.y());
        }

        const QPointF fromStart = point - start;
        // 通过投影系数 t 把点投到线段上，并裁剪到 [0, 1] 范围内。
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
        // 空对象默认按折线处理，避免上层还要额外判空。
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
        // 当前二维绘图统一落在世界坐标 Z=0 平面。
        return QVector3D(point.x(), point.y(), 0.0f);
    }
}
