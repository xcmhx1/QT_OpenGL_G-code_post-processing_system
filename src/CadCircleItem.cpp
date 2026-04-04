#include "pch.h"

#include "CadCircleItem.h"

CadCircleItem::CadCircleItem(DRW_Entity* entity, QObject* parent)
    : CadItem(entity, parent)
{
    m_data = static_cast<DRW_Circle*>(m_nativeEntity);
}

void CadCircleItem::buildGeometryDatay()
{
}

