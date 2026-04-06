#pragma once

// 声明 CadPolylineItem 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 多段线图元模块，负责多段线的离散显示和 bulge 圆弧段解释。
#include "CadItem.h"

class CadPolylineItem : public CadItem
{
public:
    explicit CadPolylineItem(DRW_Entity* entity, QObject* parent = nullptr);

    // 多段线会逐段展开；直线段直接追加端点，圆弧段则按 bulge 采样。
    void buildGeometryDatay() override;

    // 指向原生多段线实体，顶点列表和闭合标记都从这里读取。
    DRW_Polyline* m_data = nullptr;
};
