#pragma once

// 声明 CadPointItem 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 点图元模块，负责点实体的显示数据构建和基础图元属性整理。
#include "CadItem.h"

class CadPointItem : public CadItem
{
public:
    explicit CadPointItem(DRW_Entity* entity, QObject* parent = nullptr);

    // 点图元的离散结果只包含一个顶点。
    void buildGeometryDatay() override;

    // 指向原生点实体，直接读取 basePoint 作为显示位置。
    DRW_Point* m_data = nullptr;
};
