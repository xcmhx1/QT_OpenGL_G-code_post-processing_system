// 实现 CadLineItem 模块，对应头文件中声明的主要行为和协作流程。
// 直线图元模块，负责直线实体的几何离散、颜色解析和方向生成。
#include "pch.h"

#include "CadLineItem.h"

CadLineItem::CadLineItem(DRW_Entity* entity, QObject* parent)
    : CadItem(entity, parent)
{
    // 直线图元只接受 DRW_Line，这里在构造阶段完成强类型绑定。
    m_data = static_cast<DRW_Line*>(m_nativeEntity);
    // 构造后立即构建几何和方向，保证图元一进入场景即可渲染。
    buildGeometryDatay();
    buildProcessDirection();
}

void CadLineItem::buildGeometryDatay()
{
    // 重建前先清空旧几何，避免刷新实体时残留历史顶点。
    m_geometry.vertices.clear();

    if (m_data == nullptr)
    {
        return;
    }

    // 直线渲染只需要起点和终点两个顶点。
    m_geometry.vertices.reserve(2);
    m_geometry.vertices.append(QVector3D(m_data->basePoint.x, m_data->basePoint.y, m_data->basePoint.z));
    m_geometry.vertices.append(QVector3D(m_data->secPoint.x, m_data->secPoint.y, m_data->secPoint.z));
}
