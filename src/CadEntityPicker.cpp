// 实现 CadEntityPicker 模块，对应头文件中声明的主要行为和协作流程。
// 实体拾取模块，负责基于屏幕空间距离规则选择当前命中的图元。
#include "pch.h"

#include "CadEntityPicker.h"

#include "CadItem.h"
#include "CadViewerUtils.h"

#include <limits>

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
    )
    {
        const QPointF clickPoint(screenPos);
        const float maxDistanceSquared = pickThresholdPixels * pickThresholdPixels;

        EntityId bestId = 0;
        float bestDistanceSquared = maxDistanceSquared;

        for (const std::unique_ptr<CadItem>& entity : entities)
        {
            const auto& vertices = entity->m_geometry.vertices;

            if (vertices.isEmpty())
            {
                continue;
            }

            float entityDistanceSquared = std::numeric_limits<float>::max();

            if (entity->m_type == DRW::ETYPE::POINT || vertices.size() == 1)
            {
                const QPointF point = CadViewerUtils::projectToScreen
                (
                    vertices.front(),
                    viewProjection,
                    viewportWidth,
                    viewportHeight
                );
                const QPointF delta = clickPoint - point;
                entityDistanceSquared = static_cast<float>(delta.x() * delta.x() + delta.y() * delta.y());
            }
            else
            {
                for (int i = 0; i < vertices.size() - 1; ++i)
                {
                    const QPointF start = CadViewerUtils::projectToScreen
                    (
                        vertices.at(i),
                        viewProjection,
                        viewportWidth,
                        viewportHeight
                    );
                    const QPointF end = CadViewerUtils::projectToScreen
                    (
                        vertices.at(i + 1),
                        viewProjection,
                        viewportWidth,
                        viewportHeight
                    );
                    entityDistanceSquared = std::min
                    (
                        entityDistanceSquared,
                        CadViewerUtils::distanceToSegmentSquared(clickPoint, start, end)
                    );
                }
            }

            if (entityDistanceSquared <= bestDistanceSquared)
            {
                bestDistanceSquared = entityDistanceSquared;
                bestId = CadViewerUtils::toEntityId(entity.get());
            }
        }

        return bestId;
    }
}
