#pragma once

#include "CadItem.h"

class CadXlineItem : public CadItem
{
public:
    explicit CadXlineItem(DRW_Entity* entity, QObject* parent = nullptr);

    void buildGeometryDatay() override;

    void rebuildRawPathPoints3D() override;

    DRW_Xline* m_data = nullptr;
};
