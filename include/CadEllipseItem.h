#pragma once

#include "CadItem.h"

class CadEllipseItem : public CadItem
{
public:
    explicit CadEllipseItem(DRW_Entity* entity, QObject* parent = nullptr);

    void buildGeometryDatay() override;

    DRW_Ellipse* m_data = nullptr;
};
