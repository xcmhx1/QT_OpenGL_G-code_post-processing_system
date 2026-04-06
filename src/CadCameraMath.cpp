#include "pch.h"

#include "CadCameraMath.h"

#include <QMatrix3x3>

#include <cmath>

namespace
{
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kDegToRad = kPi / 180.0f;
    constexpr float kBasisEpsilon = 1.0e-8f;
    constexpr float kPoleConeHalfAngleDeg = 0.01f;
    const float kPoleConeDot = std::cos(kPoleConeHalfAngleDeg * kDegToRad);
    const QVector3D kWorldUp(0.0f, 0.0f, 1.0f);
    const QVector3D kWorldDown(0.0f, 0.0f, -1.0f);
    const QVector3D kNorthUp(0.0f, 1.0f, 0.0f);
    const QVector3D kLocalForward(0.0f, 0.0f, -1.0f);
    const QVector3D kLocalRight(1.0f, 0.0f, 0.0f);
    const QVector3D kLocalUp(0.0f, 1.0f, 0.0f);
    const QVector3D kFallbackRight(1.0f, 0.0f, 0.0f);

    struct CameraBasis
    {
        QVector3D forward;
        QVector3D right;
        QVector3D up;
    };

    CameraBasis buildCameraBasis(const QVector3D& forward, const QVector3D& preferredUp, const QVector3D& fallbackUp)
    {
        CameraBasis basis;
        basis.forward = CadCameraMath::normalizedOr(forward, kLocalForward);

        QVector3D upHint = preferredUp;
        QVector3D right = QVector3D::crossProduct(basis.forward, upHint);

        if (right.lengthSquared() <= kBasisEpsilon)
        {
            upHint = fallbackUp;
            right = QVector3D::crossProduct(basis.forward, upHint);
        }

        basis.right = CadCameraMath::normalizedOr(right, kFallbackRight);
        basis.up = CadCameraMath::normalizedOr(QVector3D::crossProduct(basis.right, basis.forward), kLocalUp);
        return basis;
    }

    QQuaternion quaternionFromBasis(const CameraBasis& basis)
    {
        const QVector3D backward = -basis.forward;

        QMatrix3x3 matrix;
        matrix(0, 0) = basis.right.x();
        matrix(1, 0) = basis.right.y();
        matrix(2, 0) = basis.right.z();
        matrix(0, 1) = basis.up.x();
        matrix(1, 1) = basis.up.y();
        matrix(2, 1) = basis.up.z();
        matrix(0, 2) = backward.x();
        matrix(1, 2) = backward.y();
        matrix(2, 2) = backward.z();

        QQuaternion orientation = QQuaternion::fromRotationMatrix(matrix);

        if (orientation.isNull())
        {
            return QQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
        }

        orientation.normalize();
        return orientation;
    }

    float quaternionDot(const QQuaternion& lhs, const QQuaternion& rhs)
    {
        return lhs.scalar() * rhs.scalar()
            + lhs.x() * rhs.x()
            + lhs.y() * rhs.y()
            + lhs.z() * rhs.z();
    }
}

namespace CadCameraMath
{
    QVector3D worldUp() { return kWorldUp; }
    QVector3D worldDown() { return kWorldDown; }
    QVector3D northUp() { return kNorthUp; }
    QVector3D localForward() { return kLocalForward; }
    QVector3D localRight() { return kLocalRight; }
    QVector3D localUp() { return kLocalUp; }
    QVector3D fallbackRight() { return kFallbackRight; }

    QVector3D normalizedOr(const QVector3D& vector, const QVector3D& fallback)
    {
        if (vector.lengthSquared() <= kBasisEpsilon)
        {
            return fallback;
        }

        QVector3D normalized = vector;
        normalized.normalize();
        return normalized;
    }

    QQuaternion buildOrientationFromForward
    (
        const QVector3D& forward,
        const QVector3D& preferredUp,
        const QVector3D& fallbackUp
    )
    {
        return quaternionFromBasis(buildCameraBasis(forward, preferredUp, fallbackUp));
    }

    QQuaternion normalizedQuaternionOrIdentity(const QQuaternion& quaternion)
    {
        QQuaternion normalized = quaternion;

        if (normalized.isNull())
        {
            return QQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
        }

        normalized.normalize();
        return normalized;
    }

    QQuaternion alignQuaternionHemisphere(const QQuaternion& previous, const QQuaternion& current)
    {
        QQuaternion normalizedPrevious = previous;
        QQuaternion normalizedCurrent = current;

        if (!normalizedPrevious.isNull())
        {
            normalizedPrevious.normalize();
        }

        if (!normalizedCurrent.isNull())
        {
            normalizedCurrent.normalize();
        }

        if (!normalizedPrevious.isNull()
            && !normalizedCurrent.isNull()
            && quaternionDot(normalizedPrevious, normalizedCurrent) < 0.0f)
        {
            return QQuaternion
            (
                -normalizedCurrent.scalar(),
                -normalizedCurrent.x(),
                -normalizedCurrent.y(),
                -normalizedCurrent.z()
            );
        }

        return normalizedCurrent;
    }

    bool violatesViewConstraint(const QQuaternion& orientation)
    {
        const QQuaternion q = normalizedQuaternionOrIdentity(orientation);
        const QVector3D forward = normalizedOr(q.rotatedVector(kLocalForward), kLocalForward);
        const float forwardUpDot = QVector3D::dotProduct(forward, kWorldUp);
        return forwardUpDot >= kPoleConeDot || forwardUpDot <= -kPoleConeDot;
    }
}
