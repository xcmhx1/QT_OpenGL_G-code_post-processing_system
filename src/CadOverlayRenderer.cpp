#include "pch.h"

#include "CadOverlayRenderer.h"

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

    void bindVec3VertexAttribute()
    {
        if (QOpenGLFunctions* functions = currentFunctions())
        {
            functions->glEnableVertexAttribArray(0);
            functions->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);
        }
    }
}

void CadOverlayRenderer::initializeOrbitMarkerBuffer()
{
    const QVector3D initialPoint(0.0f, 0.0f, 0.0f);

    m_orbitMarkerVao.create();
    m_orbitMarkerVao.bind();

    m_orbitMarkerVbo.create();
    m_orbitMarkerVbo.bind();
    m_orbitMarkerVbo.allocate(&initialPoint, static_cast<int>(sizeof(QVector3D)));

    bindVec3VertexAttribute();

    m_orbitMarkerVbo.release();
    m_orbitMarkerVao.release();
}

void CadOverlayRenderer::initializeTransientBuffer()
{
    const QVector3D initialPoint(0.0f, 0.0f, 0.0f);

    m_transientVao.create();
    m_transientVao.bind();

    m_transientVbo.create();
    m_transientVbo.bind();
    m_transientVbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    m_transientVbo.allocate(&initialPoint, static_cast<int>(sizeof(QVector3D)));

    bindVec3VertexAttribute();

    m_transientVbo.release();
    m_transientVao.release();
}

void CadOverlayRenderer::destroy()
{
    m_orbitMarkerVao.destroy();
    m_orbitMarkerVbo.destroy();
    m_transientVao.destroy();
    m_transientVbo.destroy();
}

bool CadOverlayRenderer::orbitMarkerReady() const
{
    return m_orbitMarkerVao.isCreated() && m_orbitMarkerVbo.isCreated();
}

bool CadOverlayRenderer::transientReady() const
{
    return m_transientVao.isCreated() && m_transientVbo.isCreated();
}

void CadOverlayRenderer::renderOrbitMarker
(
    QOpenGLShaderProgram& shader,
    const QMatrix4x4& mvp,
    const QVector3D& orbitCenter,
    bool visible
)
{
    if (!visible || !orbitMarkerReady())
    {
        return;
    }

    QOpenGLFunctions* functions = currentFunctions();

    if (functions == nullptr)
    {
        return;
    }

    m_orbitMarkerVbo.bind();
    m_orbitMarkerVbo.write(0, &orbitCenter, static_cast<int>(sizeof(QVector3D)));
    m_orbitMarkerVbo.release();

    shader.bind();
    shader.setUniformValue("uMvp", mvp);
    shader.setUniformValue("uColor", QVector3D(0.10f, 0.95f, 0.25f));
    shader.setUniformValue("uPointSize", 14.0f);
    shader.setUniformValue("uRoundPoint", 1);

    functions->glDisable(GL_DEPTH_TEST);

    m_orbitMarkerVao.bind();
    functions->glDrawArrays(GL_POINTS, 0, 1);
    m_orbitMarkerVao.release();

    functions->glEnable(GL_DEPTH_TEST);
    shader.release();
}

void CadOverlayRenderer::renderTransientPrimitives
(
    QOpenGLShaderProgram& shader,
    const QMatrix4x4& mvp,
    const std::vector<TransientPrimitive>& commandPrimitives,
    const std::vector<TransientPrimitive>& crosshairPrimitives
)
{
    if (!transientReady() || (commandPrimitives.empty() && crosshairPrimitives.empty()))
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
    m_transientVao.bind();

    auto drawPrimitives = [this, &shader, functions](const std::vector<TransientPrimitive>& primitives)
    {
        for (const TransientPrimitive& primitive : primitives)
        {
            if (primitive.vertices.isEmpty())
            {
                continue;
            }

            m_transientVbo.bind();
            m_transientVbo.allocate
            (
                primitive.vertices.constData(),
                primitive.vertices.size() * static_cast<int>(sizeof(QVector3D))
            );

            shader.setUniformValue("uColor", primitive.color);
            shader.setUniformValue("uPointSize", primitive.pointSize);
            shader.setUniformValue("uRoundPoint", primitive.roundPoint ? 1 : 0);

            functions->glDrawArrays(primitive.primitiveType, 0, primitive.vertices.size());
            m_transientVbo.release();
        }
    };

    if (!commandPrimitives.empty())
    {
        functions->glDisable(GL_DEPTH_TEST);
        functions->glLineWidth(2.0f);
        drawPrimitives(commandPrimitives);
    }

    if (!crosshairPrimitives.empty())
    {
        functions->glEnable(GL_DEPTH_TEST);
        functions->glLineWidth(1.0f);
        drawPrimitives(crosshairPrimitives);
    }

    m_transientVao.release();
    functions->glLineWidth(1.0f);
    functions->glEnable(GL_DEPTH_TEST);
    shader.release();
}
