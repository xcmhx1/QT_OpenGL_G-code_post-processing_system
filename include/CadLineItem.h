#pragma once

#include "CadItem.h"

class CadLineItem : public CadItem
{
public:
    explicit CadLineItem(DRW_Entity* entity, QObject* parent = nullptr);

    // 生成几何体
    void buildGeometryDatay();

    // 直线数据指针
    DRW_Line* m_data;
};

