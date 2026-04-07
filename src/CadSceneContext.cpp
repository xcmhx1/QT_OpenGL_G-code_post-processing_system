// CadSceneContext 实现文件
// 实现 CadSceneContext 模块，对应头文件中声明的主要行为和协作流程。
// 场景上下文模块，负责文档绑定、场景边界和实体遍历等基础上下文能力。
#include "pch.h"

#include "CadSceneContext.h"

#include "CadItem.h"

#include <algorithm>
#include <limits>

// 析构时断开已绑定的文档信号连接
CadSceneContext::~CadSceneContext()
{
    if (m_document != nullptr)
    {
        QObject::disconnect(m_sceneChangedConnection);
    }
}

// 获取当前绑定的文档对象
// @return 当前文档指针，未绑定时返回 nullptr
CadDocument* CadSceneContext::document() const
{
    return m_document;
}

// 刷新场景包围盒与轨道中心
void CadSceneContext::refreshBounds()
{
    // 每次刷新前先清空旧状态，只有成功扫描到有效顶点才重新置为 true
    m_hasBounds = false;

    if (m_document == nullptr || m_document->m_entities.empty())
    {
        return;
    }

    QVector3D minPoint
    (
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max()
    );

    QVector3D maxPoint
    (
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max()
    );

    for (const std::unique_ptr<CadItem>& entity : m_document->m_entities)
    {
        // 直接遍历离散后的几何顶点，得到当前场景的轴对齐包围盒
        for (const QVector3D& vertex : entity->m_geometry.vertices)
        {
            minPoint.setX(std::min(minPoint.x(), vertex.x()));
            minPoint.setY(std::min(minPoint.y(), vertex.y()));
            minPoint.setZ(std::min(minPoint.z(), vertex.z()));

            maxPoint.setX(std::max(maxPoint.x(), vertex.x()));
            maxPoint.setY(std::max(maxPoint.y(), vertex.y()));
            maxPoint.setZ(std::max(maxPoint.z(), vertex.z()));
        }
    }

    // 没有任何有效顶点时不产生包围盒
    if (minPoint.x() > maxPoint.x())
    {
        return;
    }

    m_minPoint = minPoint;
    m_maxPoint = maxPoint;
    m_orbitCenter = (m_minPoint + m_maxPoint) * 0.5f;
    m_hasBounds = true;
}

// 查询当前场景是否有有效包围盒
// @return 如果已有有效场景边界返回 true，否则返回 false
bool CadSceneContext::hasBounds() const
{
    return m_hasBounds;
}

// 获取场景包围盒最小点
// @return 包围盒最小点引用
const QVector3D& CadSceneContext::minPoint() const
{
    return m_minPoint;
}

// 获取场景包围盒最大点
// @return 包围盒最大点引用
const QVector3D& CadSceneContext::maxPoint() const
{
    return m_maxPoint;
}

// 获取轨道观察中心
// @return 轨道观察中心引用
const QVector3D& CadSceneContext::orbitCenter() const
{
    return m_orbitCenter;
}
