#pragma once

#include "CadItem.h"

class CadSplineItem : public CadItem
{
public:
    explicit CadSplineItem(DRW_Entity* entity, QObject* parent = nullptr);

    void buildGeometryDatay() override;
    void rebuildRawPathPoints3D() override;

    DRW_Spline* m_data = nullptr;
};
