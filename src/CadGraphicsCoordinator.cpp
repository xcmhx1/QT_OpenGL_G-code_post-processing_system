#include "pch.h"

#include "CadGraphicsCoordinator.h"

void CadGraphicsCoordinator::initialize()
{
    CadOpenGLState::initializeDefaults();
    m_shaderManager.initialize();
    m_referenceRenderer.initializeGridBuffer();
    m_referenceRenderer.initializeAxisBuffer();
    m_overlayRenderer.initializeOrbitMarkerBuffer();
    m_overlayRenderer.initializeTransientBuffer();
    m_initialized = true;
}

void CadGraphicsCoordinator::destroy()
{
    m_overlayRenderer.destroy();
    m_referenceRenderer.destroy();
    m_shaderManager.destroy();
    m_initialized = false;
}

bool CadGraphicsCoordinator::isInitialized() const
{
    return m_initialized;
}

void CadGraphicsCoordinator::prepareFrame(int framebufferWidth, int framebufferHeight) const
{
    CadOpenGLState::prepareFrame(framebufferWidth, framebufferHeight);
}

QOpenGLShaderProgram& CadGraphicsCoordinator::generalShader()
{
    return m_shaderManager.generalShader();
}

void CadGraphicsCoordinator::renderGrid(const QMatrix4x4& mvp)
{
    m_referenceRenderer.renderGrid(m_shaderManager.generalShader(), mvp);
}

void CadGraphicsCoordinator::renderAxis(const QMatrix4x4& mvp, bool axesSwapped)
{
    m_referenceRenderer.renderAxis(m_shaderManager.generalShader(), mvp, axesSwapped);
}

void CadGraphicsCoordinator::renderOrbitMarker(const QMatrix4x4& mvp, const QVector3D& orbitCenter, bool visible)
{
    m_overlayRenderer.renderOrbitMarker(m_shaderManager.generalShader(), mvp, orbitCenter, visible);
}

void CadGraphicsCoordinator::renderTransientPrimitives
(
    const QMatrix4x4& mvp,
    const std::vector<TransientPrimitive>& commandPrimitives,
    const std::vector<TransientPrimitive>& crosshairPrimitives
)
{
    m_overlayRenderer.renderTransientPrimitives
    (
        m_shaderManager.generalShader(),
        mvp,
        commandPrimitives,
        crosshairPrimitives
    );
}
