#pragma once

#include "CadItem.h"

class CadArcItem : public CadItem
{
public:
    explicit CadArcItem(DRW_Entity* entity, QObject* parent = nullptr);

    void buildGeometryDatay() override;

    DRW_Arc* m_data = nullptr;
};
