// CadCameraMath 实现文件
// 实现 CadCameraMath 模块，对应头文件中声明的主要行为和协作流程。
// 相机数学模块，提供视图矩阵、投影矩阵和几何计算所需的底层公式。
#include "pch.h"

#include "CadCameraMath.h"

#include <QMatrix3x3>

#include <cmath>

namespace
{
    // 数学常量和基础坐标轴约定
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
        // 前向
        QVector3D forward;

        // 右向
        QVector3D right;

        // 上向
        QVector3D up;
    };

    // 根据前向与上方向构建一组正交相机基
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

    // 根据正交基生成对应的朝向四元数
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

    // 计算四元数点积，用于判断是否跨半球
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
    // 获取世界坐标系向上方向
    QVector3D worldUp() { return kWorldUp; }

    // 获取世界坐标系向下方向
    QVector3D worldDown() { return kWorldDown; }

    // 获取顶视图约定的上方向
    QVector3D northUp() { return kNorthUp; }

    // 获取相机局部前向
    QVector3D localForward() { return kLocalForward; }

    // 获取相机局部右向
    QVector3D localRight() { return kLocalRight; }

    // 获取相机局部上向
    QVector3D localUp() { return kLocalUp; }

    // 获取回退右向量
    QVector3D fallbackRight() { return kFallbackRight; }

    // 安全归一化向量
    // @param vector 输入向量
    // @param fallback 输入退化时的回退向量
    // @return 归一化结果或回退向量
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

    // 根据前向与上方向构建相机朝向
    QQuaternion buildOrientationFromForward
    (
        const QVector3D& forward,
        const QVector3D& preferredUp,
        const QVector3D& fallbackUp
    )
    {
        return quaternionFromBasis(buildCameraBasis(forward, preferredUp, fallbackUp));
    }

    // 归一化四元数，空四元数时返回单位四元数
    // @param quaternion 输入四元数
    // @return 归一化结果
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

    // 对齐四元数半球，避免插值和比较中的符号翻转
    // @param previous 上一个朝向
    // @param current 当前候选朝向
    // @return 对齐后的当前朝向
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

    // 判断朝向是否违反视图极角约束
    // @param orientation 待检测的相机朝向
    // @return 如果违反约束返回 true，否则返回 false
    bool violatesViewConstraint(const QQuaternion& orientation)
    {
        const QQuaternion q = normalizedQuaternionOrIdentity(orientation);
        const QVector3D forward = normalizedOr(q.rotatedVector(kLocalForward), kLocalForward);
        const float forwardUpDot = QVector3D::dotProduct(forward, kWorldUp);
        return forwardUpDot >= kPoleConeDot || forwardUpDot <= -kPoleConeDot;
    }
}
