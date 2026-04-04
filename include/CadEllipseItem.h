#pragma once

#include "CadItem.h"

class CadEllipseItem : public CadItem
{
public:
    explicit CadEllipseItem(DRW_Entity* entity, QObject* parent = nullptr);

    DRW_Ellipse* m_data;

    void buildGeometryDatay();
};
