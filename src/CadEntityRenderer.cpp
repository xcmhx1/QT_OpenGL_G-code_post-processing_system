// 实现 CadEntityRenderer 模块，对应头文件中声明的主要行为和协作流程。
// 实体渲染模块，负责把缓存好的图元数据提交给 OpenGL 绘制。
#include "pch.h"

#include "CadEntityRenderer.h"

#include "CadItem.h"
#include "CadSceneRenderCache.h"
#include "CadViewerUtils.h"

#include <QOpenGLContext>
#include <QOpenGLFunctions>

namespace
{
    QOpenGLFunctions* currentFunctions()
    {
        if (QOpenGLContext* context = QOpenGLContext::currentContext())
        {
            return context->functions();
        }

        return nullptr;
    }
}

void CadEntityRenderer::renderEntities
(
    QOpenGLShaderProgram& shader,
    const QMatrix4x4& mvp,
    const std::vector<std::unique_ptr<CadItem>>& entities,
    CadSceneRenderCache& sceneRenderCache,
    EntityId selectedEntityId
)
{
    QOpenGLFunctions* functions = currentFunctions();

    if (functions == nullptr)
    {
        return;
    }

    shader.bind();
    shader.setUniformValue("uMvp", mvp);
    shader.setUniformValue("uRoundPoint", 0);

    auto& entityBuffers = sceneRenderCache.entityBuffers();

    for (const std::unique_ptr<CadItem>& entity : entities)
    {
        const EntityId id = CadViewerUtils::toEntityId(entity.get());
        const auto it = entityBuffers.find(id);

        if (it == entityBuffers.end())
        {
            continue;
        }

        EntityGpuBuffer& buffer = it->second;
        const bool isSelected = id == selectedEntityId;
        const QVector3D color = isSelected ? QVector3D(1.0f, 0.80f, 0.15f) : buffer.color;
        const float pointSize = buffer.primitiveType == GL_POINTS ? (isSelected ? 12.0f : 8.0f) : 1.0f;

        shader.setUniformValue("uColor", color);
        shader.setUniformValue("uPointSize", pointSize);

        buffer.vao.bind();
        functions->glDrawArrays(buffer.primitiveType, 0, buffer.vertexCount);
        buffer.vao.release();
    }

    shader.release();
}
