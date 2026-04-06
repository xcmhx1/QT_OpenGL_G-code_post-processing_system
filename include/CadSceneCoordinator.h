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
        // 场景切换时重绑文档信号，并强制后续重建一次 GPU 缓冲。
        m_sceneContext.bindDocument(document, receiver, method);
        m_buffersDirty = true;
    }

    // 刷新场景包围盒，供相机 fit 和轨道中心计算使用。
    void refreshBounds();
    // 标记缓存失效，下一帧会触发重建。
    void markBuffersDirty();
    bool buffersDirty() const;
    // 在 OpenGL 已初始化的前提下，保证当前场景的 GPU 缓冲可用。
    void ensureGpuBuffersReady(bool graphicsInitialized);
    // 清空所有场景级 GPU 缓冲。
    void clearAllBuffers();

    // 下列接口主要是把 scene context / render cache 对外做统一转发。
    CadDocument* document() const;
    bool hasBounds() const;
    const QVector3D& minPoint() const;
    const QVector3D& maxPoint() const;
    const QVector3D& orbitCenter() const;

    CadSceneRenderCache& renderCache();
    const CadSceneRenderCache& renderCache() const;
    CadItem* findEntityById(EntityId id) const;

private:
    // scene context 管理文档绑定、边界和基础场景查询。
    CadSceneContext m_sceneContext;
    // render cache 管理实体对应的 GPU 资源。
    CadSceneRenderCache m_sceneRenderCache;
    // 只要文档变化或切换，就置脏等待下一次重建。
    bool m_buffersDirty = true;
};
