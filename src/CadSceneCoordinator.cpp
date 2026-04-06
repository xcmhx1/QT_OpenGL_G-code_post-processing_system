// 实现 CadSceneCoordinator 模块，对应头文件中声明的主要行为和协作流程。
// 场景协调模块，负责包围盒刷新、GPU 缓冲重建和场景级查询。
#include "pch.h"

#include "CadSceneCoordinator.h"

#include "CadItem.h"
#include "CadViewerUtils.h"

void CadSceneCoordinator::refreshBounds()
{
    m_sceneContext.refreshBounds();
}

void CadSceneCoordinator::markBuffersDirty()
{
    m_buffersDirty = true;
}

bool CadSceneCoordinator::buffersDirty() const
{
    return m_buffersDirty;
}

void CadSceneCoordinator::ensureGpuBuffersReady(bool graphicsInitialized)
{
    if (!m_buffersDirty)
    {
        return;
    }

    m_sceneRenderCache.clearAllBuffers();
    m_sceneContext.refreshBounds();

    CadDocument* scene = m_sceneContext.document();

    if (!graphicsInitialized)
    {
        m_buffersDirty = scene != nullptr;
        return;
    }

    if (scene == nullptr)
    {
        m_buffersDirty = false;
        return;
    }

    m_sceneRenderCache.rebuildAllBuffers(scene->m_entities);
    m_buffersDirty = false;
}

void CadSceneCoordinator::clearAllBuffers()
{
    m_sceneRenderCache.clearAllBuffers();
    m_buffersDirty = false;
}

CadDocument* CadSceneCoordinator::document() const
{
    return m_sceneContext.document();
}

bool CadSceneCoordinator::hasBounds() const
{
    return m_sceneContext.hasBounds();
}

const QVector3D& CadSceneCoordinator::minPoint() const
{
    return m_sceneContext.minPoint();
}

const QVector3D& CadSceneCoordinator::maxPoint() const
{
    return m_sceneContext.maxPoint();
}

const QVector3D& CadSceneCoordinator::orbitCenter() const
{
    return m_sceneContext.orbitCenter();
}

CadSceneRenderCache& CadSceneCoordinator::renderCache()
{
    return m_sceneRenderCache;
}

const CadSceneRenderCache& CadSceneCoordinator::renderCache() const
{
    return m_sceneRenderCache;
}

CadItem* CadSceneCoordinator::findEntityById(EntityId id) const
{
    CadDocument* scene = m_sceneContext.document();

    if (id == 0 || scene == nullptr)
    {
        return nullptr;
    }

    for (const std::unique_ptr<CadItem>& entity : scene->m_entities)
    {
        if (CadViewerUtils::toEntityId(entity.get()) == id)
        {
            return entity.get();
        }
    }

    return nullptr;
}
