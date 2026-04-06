#include "pch.h"

#include "CadCamera.h"

#include "CadCameraMath.h"

#include <algorithm>

QVector3D OrbitalCamera::eyePosition() const
{
    return target - forwardDirection() * distance;
}

QVector3D OrbitalCamera::forwardDirection() const
{
    return CadCameraMath::normalizedOr
    (
        CadCameraMath::normalizedQuaternionOrIdentity(orientation).rotatedVector(CadCameraMath::localForward()),
        CadCameraMath::localForward()
    );
}

QVector3D OrbitalCamera::rightDirection() const
{
    return CadCameraMath::normalizedOr
    (
        CadCameraMath::normalizedQuaternionOrIdentity(orientation).rotatedVector(CadCameraMath::localRight()),
        CadCameraMath::localRight()
    );
}

QVector3D OrbitalCamera::upDirection() const
{
    return CadCameraMath::normalizedOr
    (
        CadCameraMath::normalizedQuaternionOrIdentity(orientation).rotatedVector(CadCameraMath::localUp()),
        CadCameraMath::localUp()
    );
}

QMatrix4x4 OrbitalCamera::viewMatrix() const
{
    QMatrix4x4 matrix;
    matrix.lookAt(eyePosition(), target, upDirection());
    return matrix;
}

QMatrix4x4 OrbitalCamera::projectionMatrix(float aspectRatio) const
{
    const float safeAspectRatio = std::max(0.001f, aspectRatio);
    const float halfHeight = viewHeight * 0.5f;
    const float halfWidth = halfHeight * safeAspectRatio;

    QMatrix4x4 matrix;
    matrix.ortho(-halfWidth, halfWidth, -halfHeight, halfHeight, nearPlane, farPlane);
    return matrix;
}

QMatrix4x4 OrbitalCamera::viewProjectionMatrix(float aspectRatio) const
{
    return projectionMatrix(aspectRatio) * viewMatrix();
}

void OrbitalCamera::orbit(float deltaAzimuth, float deltaElevation)
{
    const QQuaternion previousOrientation = CadCameraMath::normalizedQuaternionOrIdentity(orientation);
    QQuaternion candidateOrientation = previousOrientation;

    if (!qFuzzyIsNull(deltaAzimuth))
    {
        QQuaternion yawOrientation =
            QQuaternion::fromAxisAndAngle(CadCameraMath::worldUp(), deltaAzimuth) * candidateOrientation;
        yawOrientation.normalize();

        if (!CadCameraMath::violatesViewConstraint(yawOrientation))
        {
            candidateOrientation = yawOrientation;
        }
    }

    if (!qFuzzyIsNull(deltaElevation))
    {
        const QVector3D pitchAxis = CadCameraMath::normalizedOr
        (
            candidateOrientation.rotatedVector(CadCameraMath::localRight()),
            CadCameraMath::localRight()
        );

        QQuaternion pitchOrientation =
            QQuaternion::fromAxisAndAngle(pitchAxis, deltaElevation) * candidateOrientation;
        pitchOrientation.normalize();

        if (!CadCameraMath::violatesViewConstraint(pitchOrientation))
        {
            candidateOrientation = pitchOrientation;
        }
    }

    candidateOrientation.normalize();
    orientation = CadCameraMath::alignQuaternionHemisphere(previousOrientation, candidateOrientation);
    updateAxesSwappedState();
}

void OrbitalCamera::pan(float worldDx, float worldDy)
{
    target += rightDirection() * worldDx + upDirection() * worldDy;
}

void OrbitalCamera::zoom(float factor)
{
    if (factor <= 0.0f)
    {
        return;
    }

    viewHeight = std::clamp(viewHeight / factor, kMinViewHeight, kMaxViewHeight);
}

void OrbitalCamera::zoomAtPoint(float factor, const QVector3D& worldAnchor)
{
    if (factor <= 0.0f)
    {
        return;
    }

    const QVector3D planeOffset = worldAnchor - target;
    const QVector3D forward = forwardDirection();
    const QVector3D anchorOnCameraPlane = planeOffset - QVector3D::dotProduct(planeOffset, forward) * forward;
    const QVector3D deltaTarget = anchorOnCameraPlane * (1.0f - 1.0f / factor);

    zoom(factor);
    target += deltaTarget;
}

void OrbitalCamera::fitAll(const QVector3D& sceneMin, const QVector3D& sceneMax, float aspectRatio)
{
    const QVector3D size = sceneMax - sceneMin;

    target = (sceneMin + sceneMax) * 0.5f;

    const float safeAspectRatio = std::max(0.001f, aspectRatio);
    const float width = std::max(size.x(), 1.0f);
    const float height = std::max(size.y(), 1.0f);
    const float depth = std::max(size.z(), 1.0f);

    viewHeight = std::clamp(std::max(height, width / safeAspectRatio) * 1.2f, kMinViewHeight, kMaxViewHeight);
    distance = std::max({ width, height, depth, 100.0f }) * 2.0f;

    resetTo2DTopView();
}

void OrbitalCamera::resetTo2DTopView()
{
    orientation = CadCameraMath::buildOrientationFromForward
    (
        CadCameraMath::worldDown(),
        CadCameraMath::northUp(),
        CadCameraMath::fallbackRight()
    );
    updateAxesSwappedState();
}

void OrbitalCamera::enter3DFrom2D()
{
    const QQuaternion topViewOrientation = CadCameraMath::buildOrientationFromForward
    (
        CadCameraMath::worldDown(),
        CadCameraMath::northUp(),
        CadCameraMath::fallbackRight()
    );
    const QQuaternion yawOffset = QQuaternion::fromAxisAndAngle(CadCameraMath::worldUp(), 0.0f);
    const QQuaternion pitchOffset = QQuaternion::fromAxisAndAngle(QVector3D(1.0f, 0.0f, 0.0f), 0.0f);
    orientation = yawOffset * pitchOffset * topViewOrientation;
    orientation.normalize();
    updateAxesSwappedState();
}

void OrbitalCamera::updateAxesSwappedState()
{
    m_axesSwapped = forwardDirection().z() > 0.0f;
}
