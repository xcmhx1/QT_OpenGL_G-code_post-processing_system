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
    // 把对象地址稳定映射为实体 ID。
    // 当前实现直接使用指针值，因此只在对象生命周期内有效。
    EntityId toEntityId(const CadItem* entity);

    // 把世界坐标投影到屏幕像素坐标，供拾取和辅助 UI 使用。
    QPointF projectToScreen
    (
        const QVector3D& worldPos,
        const QMatrix4x4& viewProjection,
        int viewportWidth,
        int viewportHeight
    );

    // 计算屏幕点到线段的最短距离平方，用于折线类实体拾取。
    float distanceToSegmentSquared(const QPointF& point, const QPointF& start, const QPointF& end);

    // 根据图元类型选择 OpenGL primitive type。
    // 点用 GL_POINTS，直线用 GL_LINES，其余连续曲线离散结果统一走 GL_LINE_STRIP。
    GLenum primitiveTypeForEntity(const CadItem* entity);

    // 把任意三维点压回 Z=0 平面，常用于二维绘图与交互兜底。
    QVector3D flattenedToGroundPlane(const QVector3D& point);
}
