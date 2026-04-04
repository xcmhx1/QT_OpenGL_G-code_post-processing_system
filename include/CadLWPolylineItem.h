#pragma once

#include "CadItem.h"

class CadLWPolylineItem : public CadItem
{
public:
    explicit CadLWPolylineItem(DRW_Entity* entity, QObject* parent = nullptr);

    void buildGeometryDatay() override;

    DRW_LWPolyline* m_data = nullptr;
};
