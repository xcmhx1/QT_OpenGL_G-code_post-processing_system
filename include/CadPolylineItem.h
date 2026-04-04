#pragma once

#include "CadItem.h"

class CadPolylineItem : public CadItem
{
public:
    explicit CadPolylineItem(DRW_Entity* entity, QObject* parent = nullptr);

    void buildGeometryDatay() override;

    DRW_Polyline* m_data = nullptr;
};
