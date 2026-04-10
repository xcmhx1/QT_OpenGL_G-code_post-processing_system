// CadEntityRenderer 实现文件
// 实现 CadEntityRenderer 模块，对应头文件中声明的主要行为和协作流程。
// 实体渲染模块，负责把缓存好的图元数据提交给 OpenGL 绘制。
#include "pch.h"

#include "CadEntityRenderer.h"

#include "CadItem.h"
#include "CadSceneRenderCache.h"
#include "CadViewerUtils.h"

#include <cmath>
#include <QOpenGLContext>
#include <QOpenGLFunctions>

namespace
{
    // 获取当前 OpenGL 上下文导出的函数表
    // @return 可用的 OpenGL 函数表指针，无上下文时返回 nullptr
    QOpenGLFunctions* currentFunctions()
    {
        // 绘制阶段总是依赖当前上下文导出的通用 OpenGL 函数表。
        if (QOpenGLContext* context = QOpenGLContext::currentContext())
        {
            return context->functions();
        }

        return nullptr;
    }

    float srgbToLinear(float value)
    {
        return value <= 0.04045f
            ? value / 12.92f
            : std::pow((value + 0.055f) / 1.055f, 2.4f);
    }

    float relativeLuminance(const QColor& color)
    {
        const float r = srgbToLinear(color.redF());
        const float g = srgbToLinear(color.greenF());
        const float b = srgbToLinear(color.blueF());
        return 0.2126f * r + 0.7152f * g + 0.0722f * b;
    }

    float contrastRatio(const QColor& first, const QColor& second)
    {
        const float luminanceA = relativeLuminance(first);
        const float luminanceB = relativeLuminance(second);
        const float lighter = std::max(luminanceA, luminanceB);
        const float darker = std::min(luminanceA, luminanceB);
        return (lighter + 0.05f) / (darker + 0.05f);
    }

    QVector3D toVector3D(const QColor& color)
    {
        return QVector3D(color.redF(), color.greenF(), color.blueF());
    }

    QVector3D resolveDisplayColor(const QVector3D& rawColor, const AppThemeColors& theme)
    {
        QColor displayColor = QColor::fromRgbF(rawColor.x(), rawColor.y(), rawColor.z());

        if (!theme.dark && contrastRatio(displayColor, theme.viewerBackgroundColor) < 2.8f)
        {
            if (displayColor.hslSaturationF() <= 0.08f)
            {
                displayColor = theme.textPrimaryColor;
            }
            else
            {
                QColor adjustedColor = displayColor;

                for (int step = 0; step < 4 && contrastRatio(adjustedColor, theme.viewerBackgroundColor) < 2.8f; ++step)
                {
                    adjustedColor = adjustedColor.darker(170);
                }

                displayColor = contrastRatio(adjustedColor, theme.viewerBackgroundColor) >= 2.8f
                    ? adjustedColor
                    : theme.textPrimaryColor;
            }
        }

        return toVector3D(displayColor);
    }
}

// 绘制场景实体
// @param shader 通用绘制 Shader
// @param mvp 当前视图使用的模型视图投影矩阵
// @param entities 场景实体列表
// @param sceneRenderCache 实体对应的 GPU 缓冲缓存
// @param selectedEntityId 当前选中的实体 ID
void CadEntityRenderer::renderEntities
(
    QOpenGLShaderProgram& shader,
    const QMatrix4x4& mvp,
    const std::vector<std::unique_ptr<CadItem>>& entities,
    CadSceneRenderCache& sceneRenderCache,
    EntityId selectedEntityId,
    const AppThemeColors& theme
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
        const bool isSelected = entity->m_isSelected || id == selectedEntityId;
        const QVector3D color = isSelected ? QVector3D(1.0f, 0.80f, 0.15f) : resolveDisplayColor(buffer.color, theme);
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
