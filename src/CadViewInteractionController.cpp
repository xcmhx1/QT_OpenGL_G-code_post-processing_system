// CadViewInteractionController 实现文件
// 实现 CadViewInteractionController 模块，对应头文件中声明的主要行为和协作流程。
// 视图交互模块，负责平移、轨道观察和顶视图切换等观察控制。

#include "pch.h"

#include "CadViewInteractionController.h"

// CAD 模块内部依赖
#include "CadCameraUtils.h"

// 适配场景后重置控制器状态
// 通常在fitScene操作后调用，用于重置交互状态
void CadViewInteractionController::resetForFitScene()
{
    m_viewMode = CameraViewMode::Planar2D;           // 重置为2D平面视图
    m_interactionMode = ViewInteractionMode::Idle;   // 重置为无交互状态
    m_ignoreNextOrbitDelta = false;                  // 重置忽略标志
}

// 开始轨道旋转交互
// @param camera 轨道相机引用，用于设置初始状态
void CadViewInteractionController::beginOrbitInteraction(OrbitalCamera& camera)
{
    // 如果当前是2D模式，则切换到3D模式
    if (m_viewMode == CameraViewMode::Planar2D)
    {
        camera.enter3DFrom2D();              // 进入3D视图模式
        m_viewMode = CameraViewMode::Orbit3D; // 更新视图模式
        m_ignoreNextOrbitDelta = true;        // 设置忽略第一次增量标志
    }
    else
    {
        m_ignoreNextOrbitDelta = false;       // 已在3D模式，无需忽略
    }

    m_interactionMode = ViewInteractionMode::Orbiting;  // 设置为轨道旋转交互状态
}

// 开始平移交互
void CadViewInteractionController::beginPanInteraction()
{
    m_interactionMode = ViewInteractionMode::Panning;  // 设置为平移交互状态
    m_ignoreNextOrbitDelta = false;                    // 重置忽略标志
}

// 更新轨道旋转交互
// 根据屏幕移动增量更新相机的轨道旋转
// @param camera 轨道相机引用
// @param orbitCenter 轨道旋转中心点
// @param hasSceneBounds 场景是否有有效边界
// @param screenDelta 屏幕坐标增量
void CadViewInteractionController::updateOrbitInteraction
(
    OrbitalCamera& camera,
    const QVector3D& orbitCenter,
    bool hasSceneBounds,
    const QPoint& screenDelta
)
{
    // 计算方位角和仰角的增量（灵敏度系数0.4，负号用于方向修正）
    const float deltaAzimuth = -screenDelta.x() * 0.4f;
    const float deltaElevation = -screenDelta.y() * 0.4f;

    // 如果没有场景边界，使用相机的默认轨道旋转
    if (!hasSceneBounds)
    {
        camera.orbit(deltaAzimuth, deltaElevation);
        return;
    }

    // 有场景边界时，围绕指定中心点进行轨道旋转
    CadCameraUtils::orbitCameraAroundPivot(camera, orbitCenter, deltaAzimuth, deltaElevation);
}

// 更新平移交互
// 根据屏幕移动增量更新相机的平移
// @param camera 轨道相机引用
// @param pixelToWorldScale 像素到世界的缩放比例
// @param screenDelta 屏幕坐标增量
void CadViewInteractionController::updatePanInteraction(OrbitalCamera& camera, float pixelToWorldScale, const QPoint& screenDelta)
{
    // 将屏幕增量转换为世界坐标增量并平移相机
    // 注意：Y方向增量取反，因为屏幕坐标Y轴向下，而世界坐标Y轴向上
    camera.pan(-screenDelta.x() * pixelToWorldScale, screenDelta.y() * pixelToWorldScale);
}

// 结束视图交互
// 将交互模式重置为空闲状态
void CadViewInteractionController::endViewInteraction()
{
    m_interactionMode = ViewInteractionMode::Idle;  // 重置为无交互状态
    m_ignoreNextOrbitDelta = false;                 // 重置忽略标志
}

// 重置到顶视图
// 将相机重置为2D顶视图
// @param camera 轨道相机引用
void CadViewInteractionController::resetToTopView(OrbitalCamera& camera)
{
    camera.resetTo2DTopView();  // 相机重置为2D顶视图
    resetForFitScene();         // 重置控制器状态
}

// 获取当前视图模式
// @return 相机视图模式
CameraViewMode CadViewInteractionController::viewMode() const
{
    return m_viewMode;
}

// 获取当前交互模式
// @return 视图交互模式
ViewInteractionMode CadViewInteractionController::interactionMode() const
{
    return m_interactionMode;
}

// 是否应该忽略下一次轨道旋转增量
// 在某些情况下（如交互开始时），第一次移动增量需要被忽略以避免跳跃
// @return 如果需要忽略则返回true
bool CadViewInteractionController::shouldIgnoreNextOrbitDelta() const
{
    return m_ignoreNextOrbitDelta;
}

// 消耗忽略下一次轨道旋转增量的标志
// 调用此函数后，shouldIgnoreNextOrbitDelta将返回false
void CadViewInteractionController::consumeIgnoreNextOrbitDelta()
{
    m_ignoreNextOrbitDelta = false;
}

// 轨道标记是否可见
// 在轨道旋转交互且有场景边界时显示轨道中心标记
// @param hasSceneBounds 场景是否有有效边界
// @return 如果轨道标记应该显示则返回true
bool CadViewInteractionController::orbitMarkerVisible(bool hasSceneBounds) const
{
    return m_interactionMode == ViewInteractionMode::Orbiting && hasSceneBounds;
}

// 十字准线是否被抑制
// 在视图交互期间，十字准线可能会被隐藏以提高交互流畅度
// @return 如果十字准线应该被隐藏则返回true
bool CadViewInteractionController::crosshairSuppressed() const
{
    return m_interactionMode == ViewInteractionMode::Orbiting;
}