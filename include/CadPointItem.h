#pragma once

#include "CadItem.h"

class CadPointItem : public CadItem
{
public:
    explicit CadPointItem(DRW_Entity* entity, QObject* parent = nullptr);

    void buildGeometryDatay() override;

    DRW_Point* m_data = nullptr;
};
