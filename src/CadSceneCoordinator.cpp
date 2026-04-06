// 实现 CadSceneCoordinator 模块，对应头文件中声明的主要行为和协作流程。
// 场景协调模块，负责包围盒刷新、GPU 缓冲重建和场景级查询。
#include "pch.h"

#include "CadSceneCoordinator.h"

#include "CadItem.h"
#include "CadViewerUtils.h"

void CadSceneCoordinator::refreshBounds()
{
    // 包围盒计算委托给 scene context，当前类只负责时机协调。
    m_sceneContext.refreshBounds();
}

void CadSceneCoordinator::markBuffersDirty()
{
    // 文档变化后先标记，避免每次小变动都立即强制上传 GPU。
    m_buffersDirty = true;
}

bool CadSceneCoordinator::buffersDirty() const
{
    return m_buffersDirty;
}

void CadSceneCoordinator::ensureGpuBuffersReady(bool graphicsInitialized)
{
    // 未置脏时说明缓存仍可复用，不做任何多余操作。
    if (!m_buffersDirty)
    {
        return;
    }

    // 重建前先清空旧缓存，并同步刷新场景边界。
    m_sceneRenderCache.clearAllBuffers();
    m_sceneContext.refreshBounds();

    CadDocument* scene = m_sceneContext.document();

    // OpenGL 资源未就绪时只能保留“待重建”状态，等 initializeGL 之后再上传。
    if (!graphicsInitialized)
    {
        m_buffersDirty = scene != nullptr;
        return;
    }

    // 没有文档时只需清空状态，不需要重建任何实体缓冲。
    if (scene == nullptr)
    {
        m_buffersDirty = false;
        return;
    }

    // 真正的上传工作由 render cache 完成，这里只负责组织时机。
    m_sceneRenderCache.rebuildAllBuffers(scene->m_entities);
    m_buffersDirty = false;
}

void CadSceneCoordinator::clearAllBuffers()
{
    // 常用于 Viewer 析构或上下文销毁前的主动清理。
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

    // 0 被保留为“未选中/无实体”，空场景也直接返回空。
    if (id == 0 || scene == nullptr)
    {
        return nullptr;
    }

    // 通过运行期实体 ID 回查真实对象，供选中状态同步使用。
    for (const std::unique_ptr<CadItem>& entity : scene->m_entities)
    {
        if (CadViewerUtils::toEntityId(entity.get()) == id)
        {
            return entity.get();
        }
    }

    return nullptr;
}
