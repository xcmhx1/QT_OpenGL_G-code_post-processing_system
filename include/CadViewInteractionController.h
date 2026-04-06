#pragma once

#include <QPoint>
#include <QVector3D>

#include "CadCamera.h"

enum class ViewInteractionMode
{
    Idle,
    Orbiting,
    Panning,
};

enum class CameraViewMode
{
    Planar2D,
    Orbit3D,
};

class CadViewInteractionController
{
public:
    void resetForFitScene();
    void beginOrbitInteraction(OrbitalCamera& camera);
    void beginPanInteraction();
    void updateOrbitInteraction
    (
        OrbitalCamera& camera,
        const QVector3D& orbitCenter,
        bool hasSceneBounds,
        const QPoint& screenDelta
    );
    void updatePanInteraction(OrbitalCamera& camera, float pixelToWorldScale, const QPoint& screenDelta);
    void endViewInteraction();
    void resetToTopView(OrbitalCamera& camera);

    CameraViewMode viewMode() const;
    ViewInteractionMode interactionMode() const;
    bool shouldIgnoreNextOrbitDelta() const;
    void consumeIgnoreNextOrbitDelta();
    bool orbitMarkerVisible(bool hasSceneBounds) const;
    bool crosshairSuppressed() const;

private:
    CameraViewMode m_viewMode = CameraViewMode::Planar2D;
    ViewInteractionMode m_interactionMode = ViewInteractionMode::Idle;
    bool m_ignoreNextOrbitDelta = false;
};
