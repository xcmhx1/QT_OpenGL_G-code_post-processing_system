// CadReferenceRenderer 实现文件
// 实现 CadReferenceRenderer 模块，对应头文件中声明的主要行为和协作流程。
// 参考图形渲染模块，负责网格、坐标轴和轨道中心等辅助元素绘制。
#include "pch.h"

#include "CadReferenceRenderer.h"

#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QVector3D>

#include <algorithm>
#include <cmath>
#include <vector>

namespace
{
    // 获取当前 OpenGL 上下文导出的函数表
    // @return 可用的 OpenGL 函数表指针，无上下文时返回 nullptr
    QOpenGLFunctions* currentFunctions()
    {
        if (QOpenGLContext* context = QOpenGLContext::currentContext())
        {
            return context->functions();
        }

        return nullptr;
    }

    // 绑定位置属性为三维浮点顶点输入
    void bindVec3VertexAttribute()
    {
        if (QOpenGLFunctions* functions = currentFunctions())
        {
            functions->glEnableVertexAttribArray(0);
            functions->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);
        }
    }

    QVector3D toVector3D(const QColor& color)
    {
        return QVector3D(color.redF(), color.greenF(), color.blueF());
    }
}

void CadReferenceRenderer::setTheme(const AppThemeColors& theme)
{
    m_gridColor = toVector3D(theme.viewerGridColor);
}

// 初始化网格顶点缓冲
void CadReferenceRenderer::initializeGridBuffer()
{
    m_gridVao.create();
    m_gridVao.bind();

    m_gridVbo.create();
    m_gridVbo.bind();
    m_gridVbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);

    const QVector3D initialPoint(0.0f, 0.0f, 0.0f);
    m_gridVbo.allocate(&initialPoint, static_cast<int>(sizeof(QVector3D)));

    bindVec3VertexAttribute();

    m_gridVbo.release();
    m_gridVao.release();
    m_gridVertexCount = 0;
}

// 初始化坐标轴顶点缓冲
void CadReferenceRenderer::initializeAxisBuffer()
{
    constexpr float axisLength = 300.0f;
    constexpr float dashLength = 18.0f;
    constexpr float gapLength = 10.0f;

    std::vector<QVector3D> vertices;
    vertices.reserve(64);

    vertices.emplace_back(0.0f, 0.0f, 0.0f);
    vertices.emplace_back(axisLength, 0.0f, 0.0f);

    // Y 轴
    vertices.emplace_back(0.0f, 0.0f, 0.0f);
    vertices.emplace_back(0.0f, axisLength, 0.0f);

    m_axisXyVertexCount = 4;

    m_axisZSolidOffset = static_cast<int>(vertices.size());
    vertices.emplace_back(0.0f, 0.0f, 0.0f);
    vertices.emplace_back(0.0f, 0.0f, axisLength);
    m_axisZSolidVertexCount = 2;

    m_axisZDashedOffset = static_cast<int>(vertices.size());

    // 交换视图轴时用虚线段模拟 Z 轴，便于区分前后关系
    for (float z = 0.0f; z < axisLength; z += dashLength + gapLength)
    {
        const float z0 = z;
        const float z1 = std::min(z + dashLength, axisLength);

        vertices.emplace_back(0.0f, 0.0f, z0);
        vertices.emplace_back(0.0f, 0.0f, z1);
    }

    m_axisZDashedVertexCount = static_cast<int>(vertices.size()) - m_axisZDashedOffset;

    m_axisVao.create();
    m_axisVao.bind();

    m_axisVbo.create();
    m_axisVbo.bind();
    m_axisVbo.allocate(vertices.data(), static_cast<int>(vertices.size() * sizeof(QVector3D)));

    bindVec3VertexAttribute();

    m_axisVbo.release();
    m_axisVao.release();
}

// 销毁参考图形相关 GPU 资源
void CadReferenceRenderer::destroy()
{
    m_gridVao.destroy();
    m_gridVbo.destroy();
    m_axisVao.destroy();
    m_axisVbo.destroy();
}

// 绘制背景网格
// @param shader 通用绘制 Shader
// @param mvp 当前视图使用的模型视图投影矩阵
void CadReferenceRenderer::renderGrid
(
    QOpenGLShaderProgram& shader,
    const QMatrix4x4& mvp,
    float minX,
    float maxX,
    float minY,
    float maxY,
    float gridStep
)
{
    if (gridStep <= 0.0f || !std::isfinite(gridStep))
    {
        return;
    }

    QOpenGLFunctions* functions = currentFunctions();

    if (functions == nullptr)
    {
        return;
    }

    const float margin = gridStep * 2.0f;
    const int startX = static_cast<int>(std::floor((minX - margin) / gridStep));
    const int endX = static_cast<int>(std::ceil((maxX + margin) / gridStep));
    const int startY = static_cast<int>(std::floor((minY - margin) / gridStep));
    const int endY = static_cast<int>(std::ceil((maxY + margin) / gridStep));

    std::vector<QVector3D> vertices;
    vertices.reserve(((endX - startX + 1) + (endY - startY + 1)) * 2);

    const float extendedMinY = minY - margin;
    const float extendedMaxY = maxY + margin;
    const float extendedMinX = minX - margin;
    const float extendedMaxX = maxX + margin;

    for (int index = startX; index <= endX; ++index)
    {
        const float x = static_cast<float>(index) * gridStep;
        vertices.emplace_back(x, extendedMinY, 0.0f);
        vertices.emplace_back(x, extendedMaxY, 0.0f);
    }

    for (int index = startY; index <= endY; ++index)
    {
        const float y = static_cast<float>(index) * gridStep;
        vertices.emplace_back(extendedMinX, y, 0.0f);
        vertices.emplace_back(extendedMaxX, y, 0.0f);
    }

    m_gridVertexCount = static_cast<int>(vertices.size());

    if (m_gridVertexCount <= 0)
    {
        return;
    }

    m_gridVbo.bind();
    m_gridVbo.allocate(vertices.data(), m_gridVertexCount * static_cast<int>(sizeof(QVector3D)));
    m_gridVbo.release();

    // 网格仅作为背景参考，不参与深度写入，避免干扰实体显示
    shader.bind();
    shader.setUniformValue("uMvp", mvp);
    shader.setUniformValue("uColor", m_gridColor);
    shader.setUniformValue("uPointSize", 1.0f);
    shader.setUniformValue("uRoundPoint", 0);

    functions->glDisable(GL_DEPTH_TEST);
    functions->glDepthMask(GL_FALSE);
    functions->glLineWidth(1.0f);

    m_gridVao.bind();
    functions->glDrawArrays(GL_LINES, 0, m_gridVertexCount);
    m_gridVao.release();

    functions->glDepthMask(GL_TRUE);
    functions->glEnable(GL_DEPTH_TEST);
    shader.release();
}

// 绘制世界坐标轴
// @param shader 通用绘制 Shader
// @param mvp 当前视图使用的模型视图投影矩阵
// @param axesSwapped 是否以虚线方式显示 Z 轴
void CadReferenceRenderer::renderAxis(QOpenGLShaderProgram& shader, const QMatrix4x4& mvp, bool axesSwapped)
{
    if (m_axisXyVertexCount <= 0 && m_axisZSolidVertexCount <= 0)
    {
        return;
    }

    QOpenGLFunctions* functions = currentFunctions();

    if (functions == nullptr)
    {
        return;
    }

    shader.bind();
    shader.setUniformValue("uMvp", mvp);
    shader.setUniformValue("uPointSize", 1.0f);
    shader.setUniformValue("uRoundPoint", 0);

    functions->glDisable(GL_DEPTH_TEST);
    functions->glLineWidth(3.0f);

    m_axisVao.bind();

    // X 轴红色
    shader.setUniformValue("uColor", QVector3D(0.95f, 0.30f, 0.25f));
    functions->glDrawArrays(GL_LINES, 0, 2);

    // Y 轴绿色
    shader.setUniformValue("uColor", QVector3D(0.25f, 0.85f, 0.35f));
    functions->glDrawArrays(GL_LINES, 2, 2);

    // Z 轴蓝色；交换轴时改为虚线以表达特殊视图状态
    shader.setUniformValue("uColor", QVector3D(0.30f, 0.55f, 0.95f));

    if (axesSwapped)
    {
        functions->glDrawArrays(GL_LINES, m_axisZDashedOffset, m_axisZDashedVertexCount);
    }
    else
    {
        functions->glDrawArrays(GL_LINES, m_axisZSolidOffset, m_axisZSolidVertexCount);
    }

    m_axisVao.release();
    functions->glLineWidth(1.0f);
    functions->glEnable(GL_DEPTH_TEST);
    shader.release();
}
