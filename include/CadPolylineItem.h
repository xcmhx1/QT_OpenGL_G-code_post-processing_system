#pragma once

#include "CadItem.h"

class CadPolylineItem : public CadItem
{
public:
    explicit CadPolylineItem(DRW_Entity* entity, QObject* parent = nullptr);

    DRW_Polyline* m_data;

    void buildGeometryDatay();
};
