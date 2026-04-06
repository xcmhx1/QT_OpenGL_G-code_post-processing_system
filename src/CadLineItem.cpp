// 实现 CadLineItem 模块，对应头文件中声明的主要行为和协作流程。
// 直线图元模块，负责直线实体的几何离散、颜色解析和方向生成。
#include "pch.h"

#include "CadLineItem.h"

CadLineItem::CadLineItem(DRW_Entity* entity, QObject* parent)
    : CadItem(entity, parent)
{
    m_data = static_cast<DRW_Line*>(m_nativeEntity);
    buildGeometryDatay();
    buildProcessDirection();
}

void CadLineItem::buildGeometryDatay()
{
    m_geometry.vertices.clear();

    if (m_data == nullptr)
    {
        return;
    }

    m_geometry.vertices.reserve(2);
    m_geometry.vertices.append(QVector3D(m_data->basePoint.x, m_data->basePoint.y, m_data->basePoint.z));
    m_geometry.vertices.append(QVector3D(m_data->secPoint.x, m_data->secPoint.y, m_data->secPoint.z));
}
