// 实现 CadSceneRenderCache 模块，对应头文件中声明的主要行为和协作流程。
// 场景渲染缓存模块，负责维护实体对应的 GPU 资源与缓存数据。
#include "pch.h"

#include "CadSceneRenderCache.h"

#include "CadItem.h"
#include "CadViewerUtils.h"

#include <QOpenGLContext>
#include <QOpenGLFunctions>

void CadSceneRenderCache::uploadEntity(const CadItem* entity)
{
    // 只有存在离散几何的图元才值得上传到 GPU。
    if (entity == nullptr || entity->m_geometry.vertices.isEmpty())
    {
        return;
    }

    const EntityId id = CadViewerUtils::toEntityId(entity);
    // 先删旧缓冲再重建，避免刷新实体时残留历史 VAO/VBO。
    removeEntityBuffer(id);

    EntityGpuBuffer& gpuBuffer = m_entityBuffers[id];
    // 这里把图元缓存转换为渲染层直接可用的顶点数、图元类型和 RGB 颜色。
    gpuBuffer.vertexCount = entity->m_geometry.vertices.size();
    gpuBuffer.primitiveType = CadViewerUtils::primitiveTypeForEntity(entity);
    gpuBuffer.color = QVector3D
    (
        entity->m_color.redF(),
        entity->m_color.greenF(),
        entity->m_color.blueF()
    );

    // 顶点数据直接按 QVector3D 连续布局写入 VBO。
    gpuBuffer.vbo.create();
    gpuBuffer.vbo.bind();
    gpuBuffer.vbo.allocate
    (
        entity->m_geometry.vertices.constData(),
        gpuBuffer.vertexCount * static_cast<int>(sizeof(QVector3D))
    );

    // 每个实体都有自己的 VAO，用于记住位置属性 0 的绑定方式。
    gpuBuffer.vao.create();
    gpuBuffer.vao.bind();

    QOpenGLFunctions* functions = QOpenGLContext::currentContext()->functions();

    // 位置属性固定使用 layout/location 0，格式为三个 float。
    if (functions != nullptr)
    {
        functions->glEnableVertexAttribArray(0);
        functions->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);
    }

    gpuBuffer.vbo.release();
    gpuBuffer.vao.release();
}

void CadSceneRenderCache::removeEntityBuffer(EntityId id)
{
    // 删除前先销毁底层 OpenGL 资源，再把缓存表项移除。
    const auto it = m_entityBuffers.find(id);

    if (it == m_entityBuffers.end())
    {
        return;
    }

    it->second.vao.destroy();
    it->second.vbo.destroy();
    m_entityBuffers.erase(it);
}

void CadSceneRenderCache::rebuildAllBuffers(const std::vector<std::unique_ptr<CadItem>>& entities)
{
    // 场景整体重建时先清空，再按当前实体列表逐个上传。
    clearAllBuffers();

    for (const std::unique_ptr<CadItem>& entity : entities)
    {
        uploadEntity(entity.get());
    }
}

void CadSceneRenderCache::clearAllBuffers()
{
    // 显式销毁 VAO/VBO，确保上下文仍有效时能及时释放 GPU 资源。
    for (auto& [id, buffer] : m_entityBuffers)
    {
        Q_UNUSED(id);
        buffer.vao.destroy();
        buffer.vbo.destroy();
    }

    m_entityBuffers.clear();
}

const std::unordered_map<EntityId, EntityGpuBuffer>& CadSceneRenderCache::entityBuffers() const
{
    return m_entityBuffers;
}

std::unordered_map<EntityId, EntityGpuBuffer>& CadSceneRenderCache::entityBuffers()
{
    return m_entityBuffers;
}
