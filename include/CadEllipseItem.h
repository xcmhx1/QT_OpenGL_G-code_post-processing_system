#pragma once

// 声明 CadEllipseItem 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 椭圆图元模块，负责椭圆实体的离散显示数据和方向信息构建。
#include "CadItem.h"

class CadEllipseItem : public CadItem
{
public:
    explicit CadEllipseItem(DRW_Entity* entity, QObject* parent = nullptr);

    // 椭圆通过长轴、短轴和参数区间采样生成折线顶点。
    void buildGeometryDatay() override;

    // 指向原生椭圆实体，长轴向量、法向和参数区间都由它提供。
    DRW_Ellipse* m_data = nullptr;
};
