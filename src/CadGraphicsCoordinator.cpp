// CadGraphicsCoordinator 实现文件
// 实现 CadGraphicsCoordinator 模块，对应头文件中声明的主要行为和协作流程。
// 渲染协调模块，负责 OpenGL 状态、Shader 以及参考图形/Overlay 渲染器的装配。
#include "pch.h"

#include "CadGraphicsCoordinator.h"

void CadGraphicsCoordinator::setTheme(const AppThemeColors& theme)
{
    m_referenceRenderer.setTheme(theme);
}

// 初始化渲染子系统
void CadGraphicsCoordinator::initialize()
{
    // 先准备默认 OpenGL 状态，再初始化各类绘制资源
    CadOpenGLState::initializeDefaults();
    m_shaderManager.initialize();
    m_referenceRenderer.initializeGridBuffer();
    m_referenceRenderer.initializeAxisBuffer();
    m_overlayRenderer.initializeOrbitMarkerBuffer();
    m_overlayRenderer.initializeTransientBuffer();
    m_initialized = true;
}

// 销毁渲染子系统
void CadGraphicsCoordinator::destroy()
{
    // 按与初始化相反的层次释放资源，避免悬空引用
    m_overlayRenderer.destroy();
    m_referenceRenderer.destroy();
    m_shaderManager.destroy();
    m_initialized = false;
}

// 查询渲染子系统是否已经完成初始化
// @return 如果已完成初始化返回 true，否则返回 false
bool CadGraphicsCoordinator::isInitialized() const
{
    return m_initialized;
}

// 为一帧绘制准备 OpenGL 状态
// @param framebufferWidth 当前帧缓冲宽度
// @param framebufferHeight 当前帧缓冲高度
void CadGraphicsCoordinator::prepareFrame(int framebufferWidth, int framebufferHeight) const
{
    CadOpenGLState::prepareFrame(framebufferWidth, framebufferHeight);
}

// 获取通用绘制 Shader
// @return 通用着色器程序引用
QOpenGLShaderProgram& CadGraphicsCoordinator::generalShader()
{
    return m_shaderManager.generalShader();
}

// 绘制背景网格
// @param mvp 当前视图使用的模型视图投影矩阵
void CadGraphicsCoordinator::renderGrid(const QMatrix4x4& mvp)
{
    m_referenceRenderer.renderGrid(m_shaderManager.generalShader(), mvp);
}

// 绘制世界坐标轴
// @param mvp 当前视图使用的模型视图投影矩阵
// @param axesSwapped 是否使用交换后的坐标轴显示策略
void CadGraphicsCoordinator::renderAxis(const QMatrix4x4& mvp, bool axesSwapped)
{
    m_referenceRenderer.renderAxis(m_shaderManager.generalShader(), mvp, axesSwapped);
}

// 绘制轨道观察中心标记
// @param mvp 当前视图使用的模型视图投影矩阵
// @param orbitCenter 轨道观察中心点
// @param visible 标记是否可见
void CadGraphicsCoordinator::renderOrbitMarker(const QMatrix4x4& mvp, const QVector3D& orbitCenter, bool visible)
{
    m_overlayRenderer.renderOrbitMarker(m_shaderManager.generalShader(), mvp, orbitCenter, visible);
}

// 绘制 transient 叠加图元
// @param mvp 当前视图使用的模型视图投影矩阵
// @param commandPrimitives 命令预览图元集合
// @param crosshairPrimitives 十字光标与拾取框图元集合
void CadGraphicsCoordinator::renderTransientPrimitives
(
    const QMatrix4x4& mvp,
    const std::vector<TransientPrimitive>& commandPrimitives,
    const std::vector<TransientPrimitive>& crosshairPrimitives
)
{
    // 两类 transient 图元共用同一个 Overlay 渲染器完成提交
    m_overlayRenderer.renderTransientPrimitives
    (
        m_shaderManager.generalShader(),
        mvp,
        commandPrimitives,
        crosshairPrimitives
    );
}
