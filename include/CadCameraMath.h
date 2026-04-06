// 声明 CadCameraMath 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 相机数学模块，提供视图矩阵、投影矩阵和几何计算所需的底层公式。
#pragma once

#include <QQuaternion>
#include <QVector3D>

namespace CadCameraMath
{
    QVector3D worldUp();
    QVector3D worldDown();
    QVector3D northUp();
    QVector3D localForward();
    QVector3D localRight();
    QVector3D localUp();
    QVector3D fallbackRight();
    QVector3D normalizedOr(const QVector3D& vector, const QVector3D& fallback);
    QQuaternion buildOrientationFromForward
    (
        const QVector3D& forward,
        const QVector3D& preferredUp,
        const QVector3D& fallbackUp
    );
    QQuaternion normalizedQuaternionOrIdentity(const QQuaternion& quaternion);
    QQuaternion alignQuaternionHemisphere(const QQuaternion& previous, const QQuaternion& current);
    bool violatesViewConstraint(const QQuaternion& orientation);
}
