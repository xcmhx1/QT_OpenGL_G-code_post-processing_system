#pragma once

#include <vector>

#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QVector3D>

#include "CadRenderTypes.h"

class CadOverlayRenderer
{
public:
    void initializeOrbitMarkerBuffer();
    void initializeTransientBuffer();
    void destroy();

    bool orbitMarkerReady() const;
    bool transientReady() const;

    void renderOrbitMarker
    (
        QOpenGLShaderProgram& shader,
        const QMatrix4x4& mvp,
        const QVector3D& orbitCenter,
        bool visible
    );

    void renderTransientPrimitives
    (
        QOpenGLShaderProgram& shader,
        const QMatrix4x4& mvp,
        const std::vector<TransientPrimitive>& commandPrimitives,
        const std::vector<TransientPrimitive>& crosshairPrimitives
    );

private:
    QOpenGLBuffer m_orbitMarkerVbo{ QOpenGLBuffer::VertexBuffer };
    QOpenGLVertexArrayObject m_orbitMarkerVao;
    QOpenGLBuffer m_transientVbo{ QOpenGLBuffer::VertexBuffer };
    QOpenGLVertexArrayObject m_transientVao;
};
