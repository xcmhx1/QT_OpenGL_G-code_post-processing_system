#pragma once

// 声明 CadPolylineItem 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 多段线图元模块，负责多段线的离散显示和 bulge 圆弧段解释。
#include "CadItem.h"

class CadPolylineItem : public CadItem
{
public:
    explicit CadPolylineItem(DRW_Entity* entity, QObject* parent = nullptr);

    void buildGeometryDatay() override;

    DRW_Polyline* m_data = nullptr;
};
