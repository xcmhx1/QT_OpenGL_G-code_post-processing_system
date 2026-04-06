#include "pch.h"

#include "CadViewInteractionController.h"

#include "CadCameraUtils.h"

void CadViewInteractionController::resetForFitScene()
{
    m_viewMode = CameraViewMode::Planar2D;
    m_interactionMode = ViewInteractionMode::Idle;
    m_ignoreNextOrbitDelta = false;
}

void CadViewInteractionController::beginOrbitInteraction(OrbitalCamera& camera)
{
    if (m_viewMode == CameraViewMode::Planar2D)
    {
        camera.enter3DFrom2D();
        m_viewMode = CameraViewMode::Orbit3D;
        m_ignoreNextOrbitDelta = true;
    }
    else
    {
        m_ignoreNextOrbitDelta = false;
    }

    m_interactionMode = ViewInteractionMode::Orbiting;
}

void CadViewInteractionController::beginPanInteraction()
{
    m_interactionMode = ViewInteractionMode::Panning;
    m_ignoreNextOrbitDelta = false;
}

void CadViewInteractionController::updateOrbitInteraction
(
    OrbitalCamera& camera,
    const QVector3D& orbitCenter,
    bool hasSceneBounds,
    const QPoint& screenDelta
)
{
    const float deltaAzimuth = -screenDelta.x() * 0.4f;
    const float deltaElevation = -screenDelta.y() * 0.4f;

    if (!hasSceneBounds)
    {
        camera.orbit(deltaAzimuth, deltaElevation);
        return;
    }

    CadCameraUtils::orbitCameraAroundPivot(camera, orbitCenter, deltaAzimuth, deltaElevation);
}

void CadViewInteractionController::updatePanInteraction(OrbitalCamera& camera, float pixelToWorldScale, const QPoint& screenDelta)
{
    camera.pan(-screenDelta.x() * pixelToWorldScale, screenDelta.y() * pixelToWorldScale);
}

void CadViewInteractionController::endViewInteraction()
{
    m_interactionMode = ViewInteractionMode::Idle;
    m_ignoreNextOrbitDelta = false;
}

void CadViewInteractionController::resetToTopView(OrbitalCamera& camera)
{
    camera.resetTo2DTopView();
    resetForFitScene();
}

CameraViewMode CadViewInteractionController::viewMode() const
{
    return m_viewMode;
}

ViewInteractionMode CadViewInteractionController::interactionMode() const
{
    return m_interactionMode;
}

bool CadViewInteractionController::shouldIgnoreNextOrbitDelta() const
{
    return m_ignoreNextOrbitDelta;
}

void CadViewInteractionController::consumeIgnoreNextOrbitDelta()
{
    m_ignoreNextOrbitDelta = false;
}

bool CadViewInteractionController::orbitMarkerVisible(bool hasSceneBounds) const
{
    return m_interactionMode == ViewInteractionMode::Orbiting && hasSceneBounds;
}

bool CadViewInteractionController::crosshairSuppressed() const
{
    return m_interactionMode == ViewInteractionMode::Orbiting;
}
