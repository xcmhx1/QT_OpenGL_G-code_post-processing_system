#include "pch.h"

#include "CadPointItem.h"

CadPointItem::CadPointItem(DRW_Entity* entity, QObject* parent)
    : CadItem(entity, parent)
{
    m_data = static_cast<DRW_Point*>(m_nativeEntity);
}

void CadPointItem::buildGeometryDatay()
{
}
