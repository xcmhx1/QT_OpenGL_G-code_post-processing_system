// CadSceneCoordinator 实现文件
// 实现 CadSceneCoordinator 模块，对应头文件中声明的主要行为和协作流程。
// 场景协调模块，负责包围盒刷新、GPU 缓冲重建和场景级查询。
#include "pch.h"

#include "CadSceneCoordinator.h"

#include "CadItem.h"
#include "CadViewerUtils.h"

// 刷新场景包围盒
void CadSceneCoordinator::refreshBounds()
{
    // 包围盒计算委托给 scene context，当前类只负责时机协调。
    m_sceneContext.refreshBounds();
}

// 标记缓存失效
void CadSceneCoordinator::markBuffersDirty()
{
    // 文档变化后先标记，避免每次小变动都立即强制上传 GPU。
    m_buffersDirty = true;
}

// 查询缓冲是否处于脏状态
// @return 如果下一帧需要重建缓冲返回 true，否则返回 false
bool CadSceneCoordinator::buffersDirty() const
{
    return m_buffersDirty;
}

// 确保当前场景的 GPU 缓冲可用
// @param graphicsInitialized 渲染子系统是否已完成 OpenGL 初始化
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

// 清空所有场景级 GPU 缓冲
void CadSceneCoordinator::clearAllBuffers()
{
    // 常用于 Viewer 析构或上下文销毁前的主动清理。
    m_sceneRenderCache.clearAllBuffers();
    m_buffersDirty = false;
}

// 获取当前绑定的文档对象
// @return 当前文档指针，未绑定时返回 nullptr
CadDocument* CadSceneCoordinator::document() const
{
    return m_sceneContext.document();
}

// 查询当前场景是否有有效包围盒
// @return 如果已有有效场景边界返回 true，否则返回 false
bool CadSceneCoordinator::hasBounds() const
{
    return m_sceneContext.hasBounds();
}

// 获取场景包围盒最小点
// @return 包围盒最小点引用
const QVector3D& CadSceneCoordinator::minPoint() const
{
    return m_sceneContext.minPoint();
}

// 获取场景包围盒最大点
// @return 包围盒最大点引用
const QVector3D& CadSceneCoordinator::maxPoint() const
{
    return m_sceneContext.maxPoint();
}

// 获取轨道观察中心
// @return 轨道观察中心引用
const QVector3D& CadSceneCoordinator::orbitCenter() const
{
    return m_sceneContext.orbitCenter();
}

// 获取场景渲染缓存
// @return 可修改的渲染缓存引用
CadSceneRenderCache& CadSceneCoordinator::renderCache()
{
    return m_sceneRenderCache;
}

// 获取场景渲染缓存
// @return 只读渲染缓存引用
const CadSceneRenderCache& CadSceneCoordinator::renderCache() const
{
    return m_sceneRenderCache;
}

// 通过实体 ID 查找对应场景对象
// @param id 实体 ID
// @return 对应实体指针，未找到时返回 nullptr
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
