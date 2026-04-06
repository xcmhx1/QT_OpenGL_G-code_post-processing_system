// 实现 CadSceneContext 模块，对应头文件中声明的主要行为和协作流程。
// 场景上下文模块，负责文档绑定、场景边界和实体遍历等基础上下文能力。
#include "pch.h"

#include "CadSceneContext.h"

#include "CadItem.h"

#include <algorithm>
#include <limits>

CadSceneContext::~CadSceneContext()
{
    if (m_document != nullptr)
    {
        QObject::disconnect(m_sceneChangedConnection);
    }
}

CadDocument* CadSceneContext::document() const
{
    return m_document;
}

void CadSceneContext::refreshBounds()
{
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

    if (minPoint.x() > maxPoint.x())
    {
        return;
    }

    m_minPoint = minPoint;
    m_maxPoint = maxPoint;
    m_orbitCenter = (m_minPoint + m_maxPoint) * 0.5f;
    m_hasBounds = true;
}

bool CadSceneContext::hasBounds() const
{
    return m_hasBounds;
}

const QVector3D& CadSceneContext::minPoint() const
{
    return m_minPoint;
}

const QVector3D& CadSceneContext::maxPoint() const
{
    return m_maxPoint;
}

const QVector3D& CadSceneContext::orbitCenter() const
{
    return m_orbitCenter;
}
