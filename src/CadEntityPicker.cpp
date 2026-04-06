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
        // 拾取逻辑统一在屏幕空间里进行，避免直接在世界空间做投影相关判断。
        const QPointF clickPoint(screenPos);
        const float maxDistanceSquared = pickThresholdPixels * pickThresholdPixels;

        EntityId bestId = 0;
        float bestDistanceSquared = maxDistanceSquared;

        for (const std::unique_ptr<CadItem>& entity : entities)
        {
            const auto& vertices = entity->m_geometry.vertices;

            // 没有离散几何的图元无法参与拾取。
            if (vertices.isEmpty())
            {
                continue;
            }

            float entityDistanceSquared = std::numeric_limits<float>::max();

            // 点图元只需比较鼠标到投影点的距离。
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
                // 折线类图元逐段比较“鼠标点到屏幕线段”的最短距离。
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

            // 只在阈值内保留最近命中的实体；后遍历到的更近对象会覆盖旧结果。
            if (entityDistanceSquared <= bestDistanceSquared)
            {
                bestDistanceSquared = entityDistanceSquared;
                bestId = CadViewerUtils::toEntityId(entity.get());
            }
        }

        return bestId;
    }
}
