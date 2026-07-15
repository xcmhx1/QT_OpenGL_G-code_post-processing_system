// CadViewer 视图导航与相机换算实现
#include "platform/pch.h"

#include "cad/view/CadViewer.h"

#include "cad/view/transform/CadViewTransform.h"

#include <array>
#include <cmath>
#include <limits>

namespace
{
    constexpr float kMinimumGridStep = 10.0f;
    constexpr float kTargetHorizontalMajorGridGroups = 5.0f;
    constexpr float kMinorGridCellsPerMajorGroup = 5.0f;
    constexpr float kXlineHandleDistancePixels = 72.0f;
}

void CadViewer::fitScene()
{
    m_sceneCoordinator.refreshBounds();

    if (!m_sceneCoordinator.hasBounds())
    {
        return;
    }

    m_camera.fitAll(m_sceneCoordinator.minPoint(), m_sceneCoordinator.maxPoint(), aspectRatio());
    m_viewInteractionController.resetForFitScene();
    m_controller.reset();
    invalidateSnapCache();
    update();
}

void CadViewer::beginOrbitInteraction()
{
    m_viewInteractionController.beginOrbitInteraction(m_camera);
    update();
}

void CadViewer::beginPanInteraction()
{
    m_viewInteractionController.beginPanInteraction();
    update();
}

void CadViewer::updateOrbitInteraction(const QPoint& screenDelta)
{
    m_viewInteractionController.updateOrbitInteraction
    (
        m_camera,
        m_sceneCoordinator.orbitCenter(),
        m_sceneCoordinator.hasBounds(),
        screenDelta
    );
    invalidateSnapCache();
    update();
}

void CadViewer::updatePanInteraction(const QPoint& screenDelta)
{
    m_viewInteractionController.updatePanInteraction(m_camera, pixelToWorldScale(), screenDelta);
    invalidateSnapCache();
    update();
}

void CadViewer::endViewInteraction()
{
    m_viewInteractionController.endViewInteraction();
}

void CadViewer::zoomAtScreenPosition(const QPoint& screenPos, float factor)
{
    const QVector3D nearPoint = screenToWorld(screenPos, -1.0f);
    const QVector3D farPoint = screenToWorld(screenPos, 1.0f);
    const QVector3D rayDirection = farPoint - nearPoint;
    const QVector3D planeNormal = m_camera.forwardDirection();

    QVector3D anchor = m_camera.target;
    const float denominator = QVector3D::dotProduct(rayDirection, planeNormal);

    if (!qFuzzyIsNull(denominator))
    {
        const float t = QVector3D::dotProduct(m_camera.target - nearPoint, planeNormal) / denominator;
        anchor = nearPoint + rayDirection * t;
    }

    m_camera.zoomAtPoint(factor, anchor);
    invalidateSnapCache();
    update();
}

void CadViewer::resetToTopView()
{
    m_viewInteractionController.resetToTopView(m_camera);
    invalidateSnapCache();
    update();
}

void CadViewer::fitSceneView()
{
    fitScene();
}

bool CadViewer::shouldIgnoreNextOrbitDelta() const
{
    return m_viewInteractionController.shouldIgnoreNextOrbitDelta();
}

void CadViewer::consumeIgnoreNextOrbitDelta()
{
    m_viewInteractionController.consumeIgnoreNextOrbitDelta();
}

void CadViewer::zoomIn(float factor)
{
    m_camera.zoom(factor);
    invalidateSnapCache();
    update();
}

void CadViewer::zoomOut(float factor)
{
    if (factor <= 0.0f)
    {
        return;
    }

    m_camera.zoom(1.0f / factor);
    invalidateSnapCache();
    update();
}

QVector3D CadViewer::screenToWorld(const QPoint& screenPos, float depth) const
{
    return CadViewTransform::screenToWorld(m_camera, m_viewportWidth, m_viewportHeight, screenPos, depth);
}

QVector3D CadViewer::screenToGroundPlane(const QPoint& screenPos) const
{
    return CadViewTransform::screenToGroundPlane(m_camera, m_viewportWidth, m_viewportHeight, screenPos);
}

QPoint CadViewer::worldToScreen(const QVector3D& worldPos) const
{
    return CadViewTransform::worldToScreen(m_camera, m_viewportWidth, m_viewportHeight, worldPos);
}

void CadViewer::computeVisibleGroundBounds(float& minX, float& maxX, float& minY, float& maxY) const
{
    minX = std::numeric_limits<float>::max();
    maxX = -std::numeric_limits<float>::max();
    minY = std::numeric_limits<float>::max();
    maxY = -std::numeric_limits<float>::max();

    const int safeWidth = std::max(1, width());
    const int safeHeight = std::max(1, height());
    const std::array<QPoint, 4> corners =
    {
        QPoint(0, 0),
        QPoint(safeWidth - 1, 0),
        QPoint(0, safeHeight - 1),
        QPoint(safeWidth - 1, safeHeight - 1)
    };

    bool hasValidCorner = false;

    for (const QPoint& corner : corners)
    {
        const QVector3D position = screenToGroundPlane(corner);

        if (!std::isfinite(position.x()) || !std::isfinite(position.y()))
        {
            continue;
        }

        minX = std::min(minX, position.x());
        maxX = std::max(maxX, position.x());
        minY = std::min(minY, position.y());
        maxY = std::max(maxY, position.y());
        hasValidCorner = true;
    }

    if (!hasValidCorner)
    {
        const float halfHeight = m_camera.viewHeight * 0.5f;
        const float halfWidth = halfHeight * aspectRatio();
        minX = m_camera.target.x() - halfWidth;
        maxX = m_camera.target.x() + halfWidth;
        minY = m_camera.target.y() - halfHeight;
        maxY = m_camera.target.y() + halfHeight;
    }
}

float CadViewer::currentGridStep() const
{
    // 网格密度只跟正交相机的缩放范围相关。若使用地平面反投影宽度，
    // 相机倾斜时交点范围会被放大，导致网格在旋转过程中产生呼吸感。
    const float stableViewWidth = std::max(m_camera.viewHeight * aspectRatio(), 0.0f);
    const float targetHorizontalCellCount = kTargetHorizontalMajorGridGroups * kMinorGridCellsPerMajorGroup;
    return std::max(stableViewWidth / targetHorizontalCellCount, kMinimumGridStep);
}

float CadViewer::aspectRatio() const
{
    return CadViewTransform::aspectRatio(m_viewportWidth, m_viewportHeight);
}

float CadViewer::pixelToWorldScale() const
{
    return CadViewTransform::pixelToWorldScale(m_camera, m_viewportHeight);
}

float CadViewer::xlineHandleWorldLength() const
{
    const float pixelScale = pixelToWorldScale();

    if (!std::isfinite(pixelScale) || pixelScale <= 0.0f)
    {
        return 50.0f;
    }

    return pixelScale * kXlineHandleDistancePixels;
}
