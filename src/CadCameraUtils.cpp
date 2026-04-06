#include "pch.h"

#include "CadCameraUtils.h"

#include "CadCameraMath.h"

namespace CadCameraUtils
{
    void orbitCameraAroundPivot
    (
        OrbitalCamera& camera,
        const QVector3D& pivot,
        float deltaAzimuth,
        float deltaElevation
    )
    {
        QVector3D eye = camera.eyePosition();
        QVector3D target = camera.target;
        const float distance = camera.distance;
        const QQuaternion previousOrientation = CadCameraMath::normalizedQuaternionOrIdentity(camera.orientation);
        QQuaternion candidateOrientation = previousOrientation;
        QVector3D candidateEye = eye;
        QVector3D candidateTarget = target;

        if (!qFuzzyIsNull(deltaAzimuth))
        {
            const QQuaternion yawRotation =
                QQuaternion::fromAxisAndAngle(QVector3D(0.0f, 0.0f, 1.0f), deltaAzimuth);

            const QVector3D yawedEye = pivot + yawRotation.rotatedVector(candidateEye - pivot);
            const QVector3D yawedTarget = pivot + yawRotation.rotatedVector(candidateTarget - pivot);
            QQuaternion yawedOrientation = yawRotation * candidateOrientation;
            yawedOrientation.normalize();

            if (!CadCameraMath::violatesViewConstraint(yawedOrientation))
            {
                candidateEye = yawedEye;
                candidateTarget = yawedTarget;
                candidateOrientation = yawedOrientation;
            }
        }

        if (!qFuzzyIsNull(deltaElevation))
        {
            const QVector3D pitchAxis = CadCameraMath::normalizedOr
            (
                candidateOrientation.rotatedVector(CadCameraMath::localRight()),
                CadCameraMath::localRight()
            );

            const QQuaternion pitchRotation =
                QQuaternion::fromAxisAndAngle(pitchAxis, deltaElevation);

            const QVector3D pitchedEye = pivot + pitchRotation.rotatedVector(candidateEye - pivot);
            const QVector3D pitchedTarget = pivot + pitchRotation.rotatedVector(candidateTarget - pivot);
            QQuaternion pitchedOrientation = pitchRotation * candidateOrientation;
            pitchedOrientation.normalize();

            if (!CadCameraMath::violatesViewConstraint(pitchedOrientation))
            {
                candidateEye = pitchedEye;
                candidateTarget = pitchedTarget;
                candidateOrientation = pitchedOrientation;
            }
        }

        candidateOrientation.normalize();
        camera.target = candidateTarget;
        camera.distance = distance;
        camera.orientation = CadCameraMath::alignQuaternionHemisphere(previousOrientation, candidateOrientation);
        camera.updateAxesSwappedState();
    }
}
