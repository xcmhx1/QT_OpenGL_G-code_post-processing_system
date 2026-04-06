#pragma once

// 声明 CadLineItem 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 直线图元模块，负责直线实体的几何离散、颜色解析和方向生成。
#include "CadItem.h"

class CadLineItem : public CadItem
{
public:
    explicit CadLineItem(DRW_Entity* entity, QObject* parent = nullptr);

    // 生成几何体
    void buildGeometryDatay() override;

    // 直线数据指针
    DRW_Line* m_data = nullptr;
};
