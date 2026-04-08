// CadGraphicsCoordinator 头文件
// 声明 CadGraphicsCoordinator 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 渲染协调模块，负责 OpenGL 状态、Shader 以及参考图形/Overlay 渲染器的装配。
#pragma once

#include <vector>

#include <QMatrix4x4>
#include <QOpenGLShaderProgram>
#include <QVector3D>

#include "CadOpenGLState.h"
#include "CadOverlayRenderer.h"
#include "CadReferenceRenderer.h"
#include "CadRenderTypes.h"
#include "CadShaderManager.h"

class CadGraphicsCoordinator
{
public:
    void setTheme(const AppThemeColors& theme);

    // 初始化渲染子系统
    // 依次建立默认 OpenGL 状态、通用 Shader，以及参考图形与 Overlay 所需的 GPU 资源
    void initialize();

    // 销毁渲染子系统
    // 释放由协调器持有和管理的各类渲染资源
    void destroy();

    // 查询渲染子系统是否已经完成初始化
    // @return 如果已完成初始化返回 true，否则返回 false
    bool isInitialized() const;

    // 为一帧绘制准备 OpenGL 状态
    // @param framebufferWidth 当前帧缓冲宽度
    // @param framebufferHeight 当前帧缓冲高度
    void prepareFrame(int framebufferWidth, int framebufferHeight) const;

    // 获取通用绘制 Shader
    // @return 通用着色器程序引用
    QOpenGLShaderProgram& generalShader();

    // 绘制背景网格
    // @param mvp 当前视图使用的模型视图投影矩阵
    void renderGrid(const QMatrix4x4& mvp);

    // 绘制世界坐标轴
    // @param mvp 当前视图使用的模型视图投影矩阵
    // @param axesSwapped 是否使用交换后的坐标轴显示策略
    void renderAxis(const QMatrix4x4& mvp, bool axesSwapped);

    // 绘制轨道观察中心标记
    // @param mvp 当前视图使用的模型视图投影矩阵
    // @param orbitCenter 轨道观察中心点
    // @param visible 标记是否可见
    void renderOrbitMarker(const QMatrix4x4& mvp, const QVector3D& orbitCenter, bool visible);

    // 绘制 transient 叠加图元
    // @param mvp 当前视图使用的模型视图投影矩阵
    // @param commandPrimitives 命令预览图元集合
    // @param crosshairPrimitives 十字光标与拾取框图元集合
    void renderTransientPrimitives
    (
        const QMatrix4x4& mvp,
        const std::vector<TransientPrimitive>& commandPrimitives,
        const std::vector<TransientPrimitive>& crosshairPrimitives
    );

private:
    // 初始化状态标记
    bool m_initialized = false;

    // 通用 Shader 管理器
    CadShaderManager m_shaderManager;

    // 网格与坐标轴渲染器
    CadReferenceRenderer m_referenceRenderer;

    // Overlay 图元渲染器
    CadOverlayRenderer m_overlayRenderer;
};
