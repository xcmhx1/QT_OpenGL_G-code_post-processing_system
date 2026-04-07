// CadCamera 头文件
// 声明 CadCamera 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 轨道相机模块，维护观察目标、缩放参数和视角切换等核心状态。
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
    // 观察目标点
    QVector3D target = { 0.0f, 0.0f, 0.0f };

    // 相机眼点到目标点的距离
    float distance = 500.0f;

    // 正交投影视口高度
    float viewHeight = 200.0f;

    // 裁剪近平面
    float nearPlane = -100000.0f;

    // 裁剪远平面
    float farPlane = 100000.0f;

    // 相机朝向四元数
    QQuaternion orientation = QQuaternion(1.0f, 0.0f, 0.0f, 0.0f);

    // 当前是否处于坐标轴交换显示状态
    bool m_axesSwapped = false;

    // 允许的最小视口高度
    static constexpr float kMinViewHeight = 0.001f;

    // 允许的最大视口高度
    static constexpr float kMaxViewHeight = 1.0e7f;

    // 获取相机眼点位置
    // @return 当前眼点世界坐标
    QVector3D eyePosition() const;

    // 获取相机前向方向
    // @return 归一化前向向量
    QVector3D forwardDirection() const;

    // 获取相机右方向
    // @return 归一化右向量
    QVector3D rightDirection() const;

    // 获取相机上方向
    // @return 归一化上向量
    QVector3D upDirection() const;

    // 构建视图矩阵
    // @return 当前相机的视图矩阵
    QMatrix4x4 viewMatrix() const;

    // 构建正交投影矩阵
    // @param aspectRatio 当前视口宽高比
    // @return 当前相机的投影矩阵
    QMatrix4x4 projectionMatrix(float aspectRatio) const;

    // 构建视图投影矩阵
    // @param aspectRatio 当前视口宽高比
    // @return 视图矩阵与投影矩阵相乘结果
    QMatrix4x4 viewProjectionMatrix(float aspectRatio) const;

    // 执行轨道旋转
    // @param deltaAzimuth 方位角增量
    // @param deltaElevation 俯仰角增量
    void orbit(float deltaAzimuth, float deltaElevation);

    // 执行平移
    // @param worldDx 沿相机右方向的世界位移
    // @param worldDy 沿相机上方向的世界位移
    void pan(float worldDx, float worldDy);

    // 按倍率缩放
    // @param factor 缩放倍率
    void zoom(float factor);

    // 以指定世界点为锚点缩放
    // @param factor 缩放倍率
    // @param worldAnchor 缩放锚点
    void zoomAtPoint(float factor, const QVector3D& worldAnchor);

    // 适配整个场景
    // @param sceneMin 场景包围盒最小点
    // @param sceneMax 场景包围盒最大点
    // @param aspectRatio 当前视口宽高比
    void fitAll(const QVector3D& sceneMin, const QVector3D& sceneMax, float aspectRatio);

    // 重置为二维顶视图
    void resetTo2DTopView();

    // 从二维顶视图进入三维视图模式
    void enter3DFrom2D();

    // 查询当前是否处于坐标轴交换显示状态
    // @return 如果处于交换显示状态返回 true，否则返回 false
    bool axesSwapped() const { return m_axesSwapped; }

    // 根据当前前向方向更新坐标轴交换状态
    void updateAxesSwappedState();
};
