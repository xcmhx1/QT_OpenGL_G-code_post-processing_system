// CadOverlayRenderer 头文件
// 声明 CadOverlayRenderer 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 叠加层渲染模块，负责命令预览、十字光标等 overlay 图元的绘制。
#pragma once

#include <vector>

#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QVector3D>

#include "CadRenderTypes.h"

class CadOverlayRenderer
{
public:
    // 初始化轨道中心标记缓冲
    void initializeOrbitMarkerBuffer();

    // 初始化 transient 图元缓冲
    void initializeTransientBuffer();

    // 销毁叠加层相关 GPU 资源
    void destroy();

    // 查询轨道中心标记缓冲是否可用
    // @return 如果缓冲已创建返回 true，否则返回 false
    bool orbitMarkerReady() const;

    // 查询 transient 图元缓冲是否可用
    // @return 如果缓冲已创建返回 true，否则返回 false
    bool transientReady() const;

    // 绘制轨道观察中心标记
    // @param shader 通用绘制 Shader
    // @param mvp 当前视图使用的模型视图投影矩阵
    // @param orbitCenter 轨道观察中心点
    // @param visible 标记是否可见
    void renderOrbitMarker
    (
        QOpenGLShaderProgram& shader,
        const QMatrix4x4& mvp,
        const QVector3D& orbitCenter,
        bool visible
    );

    // 绘制 transient 叠加图元
    // @param shader 通用绘制 Shader
    // @param mvp 当前视图使用的模型视图投影矩阵
    // @param commandPrimitives 命令预览图元集合
    // @param crosshairPrimitives 十字光标与拾取框图元集合
    void renderTransientPrimitives
    (
        QOpenGLShaderProgram& shader,
        const QMatrix4x4& mvp,
        const std::vector<TransientPrimitive>& commandPrimitives,
        const std::vector<TransientPrimitive>& crosshairPrimitives
    );

private:
    // 轨道中心标记顶点缓冲
    QOpenGLBuffer m_orbitMarkerVbo{ QOpenGLBuffer::VertexBuffer };

    // 轨道中心标记顶点数组对象
    QOpenGLVertexArrayObject m_orbitMarkerVao;

    // transient 图元顶点缓冲
    QOpenGLBuffer m_transientVbo{ QOpenGLBuffer::VertexBuffer };

    // transient 图元顶点数组对象
    QOpenGLVertexArrayObject m_transientVao;
};
