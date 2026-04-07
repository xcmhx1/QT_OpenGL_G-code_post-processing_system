// CadViewInteractionController 头文件
// 声明 CadViewInteractionController 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 视图交互模块，负责平移、轨道观察和顶视图切换等观察控制。

#pragma once

// Qt 核心模块
#include <QPoint>
#include <QVector3D>

// CAD 模块内部依赖
#include <CadCamera.h>

// 视图交互模式枚举
// 定义用户与视图交互的当前状态
enum class ViewInteractionMode
{
    Idle,       // 空闲状态，无交互
    Orbiting,   // 轨道旋转交互中
    Panning,    // 平移交互中
};

// 相机视图模式枚举
// 定义当前视图的观察模式
enum class CameraViewMode
{
    Planar2D,   // 2D平面视图模式
    Orbit3D,    // 3D轨道视图模式
};

// 视图交互控制器类：
// 管理CAD视图的交互状态，包括平移、轨道旋转和视图模式切换
// 负责处理用户输入到相机变换的映射
class CadViewInteractionController
{
public:
    // 适配场景后重置控制器状态
    // 通常在fitScene操作后调用，用于重置交互状态
    void resetForFitScene();

    // 开始轨道旋转交互
    // @param camera 轨道相机引用，用于设置初始状态
    void beginOrbitInteraction(OrbitalCamera& camera);

    // 开始平移交互
    void beginPanInteraction();

    // 更新轨道旋转交互
    // 根据屏幕移动增量更新相机的轨道旋转
    // @param camera 轨道相机引用
    // @param orbitCenter 轨道旋转中心点
    // @param hasSceneBounds 场景是否有有效边界
    // @param screenDelta 屏幕坐标增量
    void updateOrbitInteraction
    (
        OrbitalCamera& camera,
        const QVector3D& orbitCenter,
        bool hasSceneBounds,
        const QPoint& screenDelta
    );

    // 更新平移交互
    // 根据屏幕移动增量更新相机的平移
    // @param camera 轨道相机引用
    // @param pixelToWorldScale 像素到世界的缩放比例
    // @param screenDelta 屏幕坐标增量
    void updatePanInteraction(OrbitalCamera& camera, float pixelToWorldScale, const QPoint& screenDelta);

    // 结束视图交互
    // 将交互模式重置为空闲状态
    void endViewInteraction();

    // 重置到顶视图
    // 将相机重置为2D顶视图
    // @param camera 轨道相机引用
    void resetToTopView(OrbitalCamera& camera);

    // 获取当前视图模式
    // @return 相机视图模式
    CameraViewMode viewMode() const;

    // 获取当前交互模式
    // @return 视图交互模式
    ViewInteractionMode interactionMode() const;

    // 是否应该忽略下一次轨道旋转增量
    // 在某些情况下（如交互开始时），第一次移动增量需要被忽略以避免跳跃
    // @return 如果需要忽略则返回true
    bool shouldIgnoreNextOrbitDelta() const;

    // 消耗忽略下一次轨道旋转增量的标志
    // 调用此函数后，shouldIgnoreNextOrbitDelta将返回false
    void consumeIgnoreNextOrbitDelta();

    // 轨道标记是否可见
    // 在轨道旋转交互且有场景边界时显示轨道中心标记
    // @param hasSceneBounds 场景是否有有效边界
    // @return 如果轨道标记应该显示则返回true
    bool orbitMarkerVisible(bool hasSceneBounds) const;

    // 十字准线是否被抑制
    // 在视图交互期间，十字准线可能会被隐藏以提高交互流畅度
    // @return 如果十字准线应该被隐藏则返回true
    bool crosshairSuppressed() const;

private:
    // 当前视图模式
    CameraViewMode m_viewMode = CameraViewMode::Planar2D;

    // 当前交互模式
    ViewInteractionMode m_interactionMode = ViewInteractionMode::Idle;

    // 忽略下一次轨道旋转增量的标志
    bool m_ignoreNextOrbitDelta = false;
};