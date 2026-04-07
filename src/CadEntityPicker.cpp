// CadEntityPicker 实现文件
// 实现 CadEntityPicker 模块，对应头文件中声明的主要行为和协作流程。
// 实体拾取模块，负责基于屏幕空间距离规则选择当前命中的图元。

#include "pch.h"

#include "CadEntityPicker.h"

// CAD 模块内部依赖
#include "CadItem.h"
#include "CadViewerUtils.h"

// 标准库
#include <limits>

// CadEntityPicker 命名空间实现
namespace CadEntityPicker
{
    // 在屏幕空间执行简单拾取：
    // 1. 遍历所有实体
    // 2. 将实体的世界坐标通过视图投影矩阵变换到屏幕空间
    // 3. 根据实体类型计算屏幕距离：
    //    - 点图元：计算鼠标点到投影点的距离
    //    - 线/折线类图元：计算鼠标点到各投影线段的距离
    // 4. 返回在拾取阈值内且距离最近的实体ID
    // @param entities 实体列表，每个实体为唯一指针
    // @param viewProjection 视图投影矩阵，用于将世界坐标变换到裁剪空间
    // @param viewportWidth 视口宽度（像素）
    // @param viewportHeight 视口高度（像素）
    // @param screenPos 屏幕坐标点（像素）
    // @param pickThresholdPixels 拾取阈值（像素），小于此距离认为命中
    // @return 命中的实体ID，0表示未命中任何实体
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
        // 将点击点转换为浮点坐标，方便计算距离
        const QPointF clickPoint(screenPos);
        // 计算拾取阈值的平方，避免后续比较时重复开方
        const float maxDistanceSquared = pickThresholdPixels * pickThresholdPixels;

        // 初始化最佳拾取结果
        EntityId bestId = 0;  // 0表示无命中
        float bestDistanceSquared = maxDistanceSquared;  // 初始化为最大允许距离

        // 遍历所有实体
        for (const std::unique_ptr<CadItem>& entity : entities)
        {
            // 获取实体的几何顶点
            const auto& vertices = entity->m_geometry.vertices;

            // 没有离散几何的图元无法参与拾取。
            if (vertices.isEmpty())
            {
                continue;
            }

            // 初始化当前实体的最近距离为最大值
            float entityDistanceSquared = std::numeric_limits<float>::max();

            // 点图元只需比较鼠标到投影点的距离。
            if (entity->m_type == DRW::ETYPE::POINT || vertices.size() == 1)
            {
                // 将实体的第一个顶点投影到屏幕空间
                const QPointF point = CadViewerUtils::projectToScreen
                (
                    vertices.front(),
                    viewProjection,
                    viewportWidth,
                    viewportHeight
                );

                // 计算屏幕点击点到投影点的距离平方
                const QPointF delta = clickPoint - point;
                entityDistanceSquared = static_cast<float>(delta.x() * delta.x() + delta.y() * delta.y());
            }
            else
            {
                // 折线类图元逐段比较"鼠标点到屏幕线段"的最短距离。
                for (int i = 0; i < vertices.size() - 1; ++i)
                {
                    // 将线段的起点和终点投影到屏幕空间
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

                    // 计算点击点到当前线段的最短距离平方，并与之前的距离取最小值
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