#include "pch.h"

#include "CadPolylineItem.h"

CadPolylineItem::CadPolylineItem(DRW_Entity* entity, QObject* parent)
    : CadItem(entity, parent)
{
    m_data = static_cast<DRW_Polyline*>(m_nativeEntity);
}

void CadPolylineItem::buildGeometryDatay()
{
}

