#include "pch.h"

#include "CadSceneRenderCache.h"

#include "CadItem.h"
#include "CadViewerUtils.h"

#include <QOpenGLContext>
#include <QOpenGLFunctions>

void CadSceneRenderCache::uploadEntity(const CadItem* entity)
{
    if (entity == nullptr || entity->m_geometry.vertices.isEmpty())
    {
        return;
    }

    const EntityId id = CadViewerUtils::toEntityId(entity);
    removeEntityBuffer(id);

    EntityGpuBuffer& gpuBuffer = m_entityBuffers[id];
    gpuBuffer.vertexCount = entity->m_geometry.vertices.size();
    gpuBuffer.primitiveType = CadViewerUtils::primitiveTypeForEntity(entity);
    gpuBuffer.color = QVector3D
    (
        entity->m_color.redF(),
        entity->m_color.greenF(),
        entity->m_color.blueF()
    );

    gpuBuffer.vbo.create();
    gpuBuffer.vbo.bind();
    gpuBuffer.vbo.allocate
    (
        entity->m_geometry.vertices.constData(),
        gpuBuffer.vertexCount * static_cast<int>(sizeof(QVector3D))
    );

    gpuBuffer.vao.create();
    gpuBuffer.vao.bind();

    QOpenGLFunctions* functions = QOpenGLContext::currentContext()->functions();

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
    clearAllBuffers();

    for (const std::unique_ptr<CadItem>& entity : entities)
    {
        uploadEntity(entity.get());
    }
}

void CadSceneRenderCache::clearAllBuffers()
{
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
