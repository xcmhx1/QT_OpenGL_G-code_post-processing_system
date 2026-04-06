#pragma once

// 声明 CadEllipseItem 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 椭圆图元模块，负责椭圆实体的离散显示数据和方向信息构建。
#include "CadItem.h"

class CadEllipseItem : public CadItem
{
public:
    explicit CadEllipseItem(DRW_Entity* entity, QObject* parent = nullptr);

    void buildGeometryDatay() override;

    DRW_Ellipse* m_data = nullptr;
};
