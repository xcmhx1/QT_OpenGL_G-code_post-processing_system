#pragma once

// 声明 CadArcItem 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 圆弧图元模块，封装圆弧实体的几何离散、颜色解析和方向信息。
#include "CadItem.h"

class CadArcItem : public CadItem
{
public:
    explicit CadArcItem(DRW_Entity* entity, QObject* parent = nullptr);

    void buildGeometryDatay() override;

    DRW_Arc* m_data = nullptr;
};
