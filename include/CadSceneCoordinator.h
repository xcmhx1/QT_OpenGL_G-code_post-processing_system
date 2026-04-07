// CadSceneCoordinator 头文件
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

    // 刷新场景包围盒
    // 供相机 fit 和轨道中心计算使用
    void refreshBounds();

    // 标记缓存失效
    // 下一帧会触发 GPU 缓冲重建
    void markBuffersDirty();

    // 查询缓冲是否处于脏状态
    // @return 如果下一帧需要重建缓冲返回 true，否则返回 false
    bool buffersDirty() const;

    // 确保当前场景的 GPU 缓冲可用
    // @param graphicsInitialized 渲染子系统是否已完成 OpenGL 初始化
    void ensureGpuBuffersReady(bool graphicsInitialized);

    // 清空所有场景级 GPU 缓冲
    void clearAllBuffers();

    // 获取当前绑定的文档对象
    // @return 当前文档指针，未绑定时返回 nullptr
    CadDocument* document() const;

    // 查询当前场景是否有有效包围盒
    // @return 如果已有有效场景边界返回 true，否则返回 false
    bool hasBounds() const;

    // 获取场景包围盒最小点
    // @return 包围盒最小点引用
    const QVector3D& minPoint() const;

    // 获取场景包围盒最大点
    // @return 包围盒最大点引用
    const QVector3D& maxPoint() const;

    // 获取轨道观察中心
    // @return 轨道观察中心引用
    const QVector3D& orbitCenter() const;

    // 获取场景渲染缓存
    // @return 可修改的渲染缓存引用
    CadSceneRenderCache& renderCache();

    // 获取场景渲染缓存
    // @return 只读渲染缓存引用
    const CadSceneRenderCache& renderCache() const;

    // 通过实体 ID 查找对应场景对象
    // @param id 实体 ID
    // @return 对应实体指针，未找到时返回 nullptr
    CadItem* findEntityById(EntityId id) const;

private:
    // scene context 管理文档绑定、边界和基础场景查询。
    CadSceneContext m_sceneContext;
    // render cache 管理实体对应的 GPU 资源。
    CadSceneRenderCache m_sceneRenderCache;
    // 只要文档变化或切换，就置脏等待下一次重建。
    bool m_buffersDirty = true;
};
