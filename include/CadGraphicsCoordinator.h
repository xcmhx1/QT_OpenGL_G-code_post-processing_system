#pragma once

#include <vector>

#include <QMatrix4x4>
#include <QOpenGLShaderProgram>
#include <QVector3D>

#include "CadOpenGLState.h"
#include "CadOverlayRenderer.h"
#include "CadReferenceRenderer.h"
#include "CadRenderTypes.h"
#include "CadShaderManager.h"

class CadGraphicsCoordinator
{
public:
    void initialize();
    void destroy();

    bool isInitialized() const;
    void prepareFrame(int framebufferWidth, int framebufferHeight) const;
    QOpenGLShaderProgram& generalShader();

    void renderGrid(const QMatrix4x4& mvp);
    void renderAxis(const QMatrix4x4& mvp, bool axesSwapped);
    void renderOrbitMarker(const QMatrix4x4& mvp, const QVector3D& orbitCenter, bool visible);
    void renderTransientPrimitives
    (
        const QMatrix4x4& mvp,
        const std::vector<TransientPrimitive>& commandPrimitives,
        const std::vector<TransientPrimitive>& crosshairPrimitives
    );

private:
    bool m_initialized = false;
    CadShaderManager m_shaderManager;
    CadReferenceRenderer m_referenceRenderer;
    CadOverlayRenderer m_overlayRenderer;
};
