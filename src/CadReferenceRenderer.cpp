#include "pch.h"

#include "CadReferenceRenderer.h"

#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QVector3D>

#include <algorithm>
#include <vector>

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

    void bindVec3VertexAttribute()
    {
        if (QOpenGLFunctions* functions = currentFunctions())
        {
            functions->glEnableVertexAttribArray(0);
            functions->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);
        }
    }
}

void CadReferenceRenderer::initializeGridBuffer()
{
    std::vector<QVector3D> vertices;
    constexpr int gridHalfCount = 20;
    constexpr float gridStep = 100.0f;
    constexpr float gridExtent = gridHalfCount * gridStep;

    vertices.reserve((gridHalfCount * 2 + 1) * 4);

    for (int i = -gridHalfCount; i <= gridHalfCount; ++i)
    {
        const float offset = i * gridStep;
        vertices.emplace_back(offset, -gridExtent, 0.0f);
        vertices.emplace_back(offset, gridExtent, 0.0f);
        vertices.emplace_back(-gridExtent, offset, 0.0f);
        vertices.emplace_back(gridExtent, offset, 0.0f);
    }

    m_gridVertexCount = static_cast<int>(vertices.size());

    m_gridVao.create();
    m_gridVao.bind();

    m_gridVbo.create();
    m_gridVbo.bind();
    m_gridVbo.allocate(vertices.data(), m_gridVertexCount * static_cast<int>(sizeof(QVector3D)));

    bindVec3VertexAttribute();

    m_gridVbo.release();
    m_gridVao.release();
}

void CadReferenceRenderer::initializeAxisBuffer()
{
    constexpr float axisLength = 300.0f;
    constexpr float dashLength = 18.0f;
    constexpr float gapLength = 10.0f;

    std::vector<QVector3D> vertices;
    vertices.reserve(64);

    vertices.emplace_back(0.0f, 0.0f, 0.0f);
    vertices.emplace_back(axisLength, 0.0f, 0.0f);

    vertices.emplace_back(0.0f, 0.0f, 0.0f);
    vertices.emplace_back(0.0f, axisLength, 0.0f);

    m_axisXyVertexCount = 4;

    m_axisZSolidOffset = static_cast<int>(vertices.size());
    vertices.emplace_back(0.0f, 0.0f, 0.0f);
    vertices.emplace_back(0.0f, 0.0f, axisLength);
    m_axisZSolidVertexCount = 2;

    m_axisZDashedOffset = static_cast<int>(vertices.size());

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

void CadReferenceRenderer::destroy()
{
    m_gridVao.destroy();
    m_gridVbo.destroy();
    m_axisVao.destroy();
    m_axisVbo.destroy();
}

void CadReferenceRenderer::renderGrid(QOpenGLShaderProgram& shader, const QMatrix4x4& mvp)
{
    if (m_gridVertexCount <= 0)
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
    shader.setUniformValue("uColor", QVector3D(0.22f, 0.24f, 0.28f));
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

    shader.setUniformValue("uColor", QVector3D(0.95f, 0.30f, 0.25f));
    functions->glDrawArrays(GL_LINES, 0, 2);

    shader.setUniformValue("uColor", QVector3D(0.25f, 0.85f, 0.35f));
    functions->glDrawArrays(GL_LINES, 2, 2);

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
