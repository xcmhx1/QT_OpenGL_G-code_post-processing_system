#pragma once

// 声明 CadLineItem 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 直线图元模块，负责直线实体的几何离散、颜色解析和方向生成。
#include "CadItem.h"

class CadLineItem : public CadItem
{
public:
    explicit CadLineItem(DRW_Entity* entity, QObject* parent = nullptr);

    // 直线只需要两个端点即可完成离散。
    void buildGeometryDatay() override;

    void rebuildRawPathPoints3D() override;

    bool rebuildControlPoints4Axis
    (
        double axisY = 0.0,
        double axisZ = 0.0,
        double judgeCenterY = 0.0,
        double judgeCenterZ = 0.0,
        bool invertAAxisDirection = false,
        double aAxisOffsetDegrees = 0.0,
        bool keepContinuousAngle = true,
        QString* errorMessage = nullptr
    ) override;

    // 缓存强类型原生实体，避免重复 static_cast。
    DRW_Line* m_data = nullptr;
};
