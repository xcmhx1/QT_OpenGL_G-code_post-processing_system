#pragma once

#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector3D>

// 轨道相机：
// - target 为观察目标点
// - distance 为相机眼点到 target 的距离
// - orientation 表示相机朝向
// - viewHeight 控制正交投影视口高度
struct OrbitalCamera
{
    QVector3D target = { 0.0f, 0.0f, 0.0f };
    float distance = 500.0f;
    float viewHeight = 200.0f;
    float nearPlane = -100000.0f;
    float farPlane = 100000.0f;
    QQuaternion orientation = QQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
    bool m_axesSwapped = false;

    static constexpr float kMinViewHeight = 0.001f;
    static constexpr float kMaxViewHeight = 1.0e7f;

    QVector3D eyePosition() const;
    QVector3D forwardDirection() const;
    QVector3D rightDirection() const;
    QVector3D upDirection() const;
    QMatrix4x4 viewMatrix() const;
    QMatrix4x4 projectionMatrix(float aspectRatio) const;
    QMatrix4x4 viewProjectionMatrix(float aspectRatio) const;
    void orbit(float deltaAzimuth, float deltaElevation);
    void pan(float worldDx, float worldDy);
    void zoom(float factor);
    void zoomAtPoint(float factor, const QVector3D& worldAnchor);
    void fitAll(const QVector3D& sceneMin, const QVector3D& sceneMax, float aspectRatio);
    void resetTo2DTopView();
    void enter3DFrom2D();
    bool axesSwapped() const { return m_axesSwapped; }
    void updateAxesSwappedState();
};
