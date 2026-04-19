#pragma once

// 声明 CadArcItem 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 圆弧图元模块，封装圆弧实体的几何离散、颜色解析和方向信息。
#include "CadItem.h"

class CadArcItem : public CadItem
{
public:
    explicit CadArcItem(DRW_Entity* entity, QObject* parent = nullptr);

    // 圆弧会按起止角离散为折线，不会保留曲线原语到渲染层。
    void buildGeometryDatay() override;

    void rebuildRawPathPoints3D() override;

    bool rebuildControlPoints4Axis
    (
        double axisY = 0.0,
        double axisZ = 0.0,
        bool invertAAxisDirection = false,
        double aAxisOffsetDegrees = 0.0,
        bool keepContinuousAngle = true,
        QString* errorMessage = nullptr
    ) override;

    // 指向原生圆弧实体，负责提供圆心、半径、法向和角度范围。
    DRW_Arc* m_data = nullptr;
};
