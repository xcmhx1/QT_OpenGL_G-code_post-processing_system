// CadCamera 实现文件
// 实现 CadCamera 模块，对应头文件中声明的主要行为和协作流程。
// 轨道相机模块，维护观察目标、缩放参数和视角切换等核心状态。
#include "pch.h"

#include "CadCamera.h"

#include "CadCameraMath.h"

#include <algorithm>

// 获取相机眼点位置
// @return 当前眼点世界坐标
QVector3D OrbitalCamera::eyePosition() const
{
    return target - forwardDirection() * distance;
}

// 获取相机前向方向
// @return 归一化前向向量
QVector3D OrbitalCamera::forwardDirection() const
{
    return CadCameraMath::normalizedOr
    (
        CadCameraMath::normalizedQuaternionOrIdentity(orientation).rotatedVector(CadCameraMath::localForward()),
        CadCameraMath::localForward()
    );
}

// 获取相机右方向
// @return 归一化右向量
QVector3D OrbitalCamera::rightDirection() const
{
    return CadCameraMath::normalizedOr
    (
        CadCameraMath::normalizedQuaternionOrIdentity(orientation).rotatedVector(CadCameraMath::localRight()),
        CadCameraMath::localRight()
    );
}

// 获取相机上方向
// @return 归一化上向量
QVector3D OrbitalCamera::upDirection() const
{
    return CadCameraMath::normalizedOr
    (
        CadCameraMath::normalizedQuaternionOrIdentity(orientation).rotatedVector(CadCameraMath::localUp()),
        CadCameraMath::localUp()
    );
}

// 构建视图矩阵
// @return 当前相机的视图矩阵
QMatrix4x4 OrbitalCamera::viewMatrix() const
{
    QMatrix4x4 matrix;
    matrix.lookAt(eyePosition(), target, upDirection());
    return matrix;
}

// 构建正交投影矩阵
// @param aspectRatio 当前视口宽高比
// @return 当前相机的投影矩阵
QMatrix4x4 OrbitalCamera::projectionMatrix(float aspectRatio) const
{
    const float safeAspectRatio = std::max(0.001f, aspectRatio);
    const float halfHeight = viewHeight * 0.5f;
    const float halfWidth = halfHeight * safeAspectRatio;

    QMatrix4x4 matrix;
    matrix.ortho(-halfWidth, halfWidth, -halfHeight, halfHeight, nearPlane, farPlane);
    return matrix;
}

// 构建视图投影矩阵
// @param aspectRatio 当前视口宽高比
// @return 视图矩阵与投影矩阵相乘结果
QMatrix4x4 OrbitalCamera::viewProjectionMatrix(float aspectRatio) const
{
    return projectionMatrix(aspectRatio) * viewMatrix();
}

// 执行轨道旋转
// @param deltaAzimuth 方位角增量
// @param deltaElevation 俯仰角增量
void OrbitalCamera::orbit(float deltaAzimuth, float deltaElevation)
{
    // 旋转时分别尝试 yaw 和 pitch，并在每一步约束极角避免翻转
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

// 执行平移
// @param worldDx 沿相机右方向的世界位移
// @param worldDy 沿相机上方向的世界位移
void OrbitalCamera::pan(float worldDx, float worldDy)
{
    target += rightDirection() * worldDx + upDirection() * worldDy;
}

// 按倍率缩放
// @param factor 缩放倍率
void OrbitalCamera::zoom(float factor)
{
    if (factor <= 0.0f)
    {
        return;
    }

    viewHeight = std::clamp(viewHeight / factor, kMinViewHeight, kMaxViewHeight);
}

// 以指定世界点为锚点缩放
// @param factor 缩放倍率
// @param worldAnchor 缩放锚点
void OrbitalCamera::zoomAtPoint(float factor, const QVector3D& worldAnchor)
{
    if (factor <= 0.0f)
    {
        return;
    }

    // 只缩放与相机平面平行的分量，保持锚点屏幕位置尽量稳定
    const QVector3D planeOffset = worldAnchor - target;
    const QVector3D forward = forwardDirection();
    const QVector3D anchorOnCameraPlane = planeOffset - QVector3D::dotProduct(planeOffset, forward) * forward;
    const QVector3D deltaTarget = anchorOnCameraPlane * (1.0f - 1.0f / factor);

    zoom(factor);
    target += deltaTarget;
}

// 适配整个场景
// @param sceneMin 场景包围盒最小点
// @param sceneMax 场景包围盒最大点
// @param aspectRatio 当前视口宽高比
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

    // 当前项目以二维绘图为主，fit 后统一回到顶视图
    resetTo2DTopView();
}

// 重置为二维顶视图
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

// 从二维顶视图进入三维视图模式
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

// 根据当前前向方向更新坐标轴交换状态
void OrbitalCamera::updateAxesSwappedState()
{
    m_axesSwapped = forwardDirection().z() > 0.0f;
}
