// 声明 CadSceneCoordinator 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 场景协调模块，负责包围盒刷新、GPU 缓冲重建和场景级查询。
#pragma once

#include "CadRenderTypes.h"
#include "CadSceneContext.h"
#include "CadSceneRenderCache.h"

class CadItem;

class CadSceneCoordinator
{
public:
    template<typename Receiver, typename Method>
    void bindDocument(CadDocument* document, Receiver* receiver, Method method)
    {
        m_sceneContext.bindDocument(document, receiver, method);
        m_buffersDirty = true;
    }

    void refreshBounds();
    void markBuffersDirty();
    bool buffersDirty() const;
    void ensureGpuBuffersReady(bool graphicsInitialized);
    void clearAllBuffers();

    CadDocument* document() const;
    bool hasBounds() const;
    const QVector3D& minPoint() const;
    const QVector3D& maxPoint() const;
    const QVector3D& orbitCenter() const;

    CadSceneRenderCache& renderCache();
    const CadSceneRenderCache& renderCache() const;
    CadItem* findEntityById(EntityId id) const;

private:
    CadSceneContext m_sceneContext;
    CadSceneRenderCache m_sceneRenderCache;
    bool m_buffersDirty = true;
};
