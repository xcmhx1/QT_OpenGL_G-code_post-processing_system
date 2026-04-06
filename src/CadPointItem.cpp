// 实现 CadPointItem 模块，对应头文件中声明的主要行为和协作流程。
// 点图元模块，负责点实体的显示数据构建和基础图元属性整理。
#include "pch.h"

#include "CadPointItem.h"

CadPointItem::CadPointItem(DRW_Entity* entity, QObject* parent)
    : CadItem(entity, parent)
{
    // 绑定原生点实体，后续所有几何数据都直接从 basePoint 提取。
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

    // 点图元没有边，只保留一个位置顶点供渲染层绘制。
    m_geometry.vertices.append(QVector3D(m_data->basePoint.x, m_data->basePoint.y, m_data->basePoint.z));
}
