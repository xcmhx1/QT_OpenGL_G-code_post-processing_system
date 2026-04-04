#include "pch.h"

#include "CadPointItem.h"

CadPointItem::CadPointItem(DRW_Entity* entity, QObject* parent)
    : CadItem(entity, parent)
{
    m_data = static_cast<DRW_Point*>(m_nativeEntity);
    buildGeometryDatay();
    buildProcessDirection();
}

void CadPointItem::buildGeometryDatay()
{
    m_geometry.vertices.clear();

    if (m_data == nullptr)
    {
        return;
    }

    m_geometry.vertices.append(QVector3D(m_data->basePoint.x, m_data->basePoint.y, m_data->basePoint.z));
}
