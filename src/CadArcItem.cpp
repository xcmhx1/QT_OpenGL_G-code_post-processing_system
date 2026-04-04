#include "pch.h"

#include "CadArcItem.h"

CadArcItem::CadArcItem(DRW_Entity* entity, QObject* parent)
    : CadItem(entity, parent)
{
    m_data = static_cast<DRW_Arc*>(m_nativeEntity);
}

void CadArcItem::buildGeometryDatay()
{
}

