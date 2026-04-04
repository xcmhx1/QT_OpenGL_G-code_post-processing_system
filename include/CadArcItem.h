#pragma once

#include "CadItem.h"

class CadArcItem : public CadItem
{
public:
    explicit CadArcItem(DRW_Entity* entity, QObject* parent = nullptr);

    DRW_Arc* m_data;

    void buildGeometryDatay();
};
