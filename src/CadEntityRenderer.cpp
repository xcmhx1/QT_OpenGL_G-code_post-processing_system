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
        // 绘制阶段总是依赖当前上下文导出的通用 OpenGL 函数表。
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

    // 没有当前上下文时不能安全发起任何绘制命令。
    if (functions == nullptr)
    {
        return;
    }

    // 这一轮绘制共享同一个 MVP，颜色和点大小则按实体逐个切换。
    shader.bind();
    shader.setUniformValue("uMvp", mvp);
    shader.setUniformValue("uRoundPoint", 0);

    auto& entityBuffers = sceneRenderCache.entityBuffers();

    for (const std::unique_ptr<CadItem>& entity : entities)
    {
        const EntityId id = CadViewerUtils::toEntityId(entity.get());
        const auto it = entityBuffers.find(id);

        // 没有 GPU 缓冲的实体说明尚未上传或已被清理，直接跳过。
        if (it == entityBuffers.end())
        {
            continue;
        }

        EntityGpuBuffer& buffer = it->second;
        // 选中实体通过单独颜色和更大的点尺寸突出显示。
        const bool isSelected = id == selectedEntityId;
        const QVector3D color = isSelected ? QVector3D(1.0f, 0.80f, 0.15f) : buffer.color;
        const float pointSize = buffer.primitiveType == GL_POINTS ? (isSelected ? 12.0f : 8.0f) : 1.0f;

        shader.setUniformValue("uColor", color);
        shader.setUniformValue("uPointSize", pointSize);

        // 每个实体使用自己的 VAO，内部已经绑定好了顶点格式和 VBO。
        buffer.vao.bind();
        functions->glDrawArrays(buffer.primitiveType, 0, buffer.vertexCount);
        buffer.vao.release();
    }

    shader.release();
}
