#pragma once

#include "CadItem.h"

class CadCircleItem : public CadItem
{
public:
    explicit CadCircleItem(DRW_Entity* entity, QObject* parent = nullptr);

    DRW_Circle* m_data;

    void buildGeometryDatay();
};
