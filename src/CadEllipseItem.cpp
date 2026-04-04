#include "pch.h"

#include "CadEllipseItem.h"

CadEllipseItem::CadEllipseItem(DRW_Entity* entity, QObject* parent)
    : CadItem(entity, parent)
{
    m_data = static_cast<DRW_Ellipse*>(m_nativeEntity);
}

void CadEllipseItem::buildGeometryDatay()
{
}
