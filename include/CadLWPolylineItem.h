#pragma once

#include "CadItem.h"

class CadLWPolylineItem : public CadItem
{
public:
    explicit CadLWPolylineItem(DRW_Entity* entity, QObject* parent = nullptr);

    DRW_LWPolyline* m_data;

    void buildGeometryDatay();
};
