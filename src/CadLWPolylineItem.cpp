#include "pch.h"

#include "CadLWPolylineItem.h"

CadLWPolylineItem::CadLWPolylineItem(DRW_Entity* entity, QObject* parent)
    : CadItem(entity, parent)
{
    m_data = static_cast<DRW_LWPolyline*>(m_nativeEntity);
}

void CadLWPolylineItem::buildGeometryDatay()
{
}

