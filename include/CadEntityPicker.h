// CadEntityPicker 头文件
// 声明 CadEntityPicker 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 实体拾取模块，负责基于屏幕空间距离规则选择当前命中的图元。

#pragma once

// 标准库
#include <memory>
#include <vector>

// Qt 核心模块
#include <QMatrix4x4>
#include <QPoint>
#include <QRectF>

// CAD 模块内部依赖
#include "CadRenderTypes.h"

// 前向声明
class CadItem;

// CadEntityPicker 命名空间：
// 包含实体拾取相关的静态函数，不维护状态。
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
    );

    // 在屏幕空间窗口内批量拾取实体：
    // - crossingSelection=true：碰选（实体与窗口有交集即命中）
    // - crossingSelection=false：包含选（实体离散顶点全部落入窗口才命中）
    std::vector<EntityId> pickEntitiesByWindow
    (
        const std::vector<std::unique_ptr<CadItem>>& entities,
        const QMatrix4x4& viewProjection,
        int viewportWidth,
        int viewportHeight,
        const QRectF& windowRect,
        bool crossingSelection
    );
}
