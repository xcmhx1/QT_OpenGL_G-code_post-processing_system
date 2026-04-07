// CadReferenceRenderer 头文件
// 声明 CadReferenceRenderer 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 参考图形渲染模块，负责网格、坐标轴和轨道中心等辅助元素绘制。
#pragma once

#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>

class CadReferenceRenderer
{
public:
    // 初始化网格顶点缓冲
    void initializeGridBuffer();

    // 初始化坐标轴顶点缓冲
    void initializeAxisBuffer();

    // 销毁参考图形相关 GPU 资源
    void destroy();

    // 绘制背景网格
    // @param shader 通用绘制 Shader
    // @param mvp 当前视图使用的模型视图投影矩阵
    void renderGrid(QOpenGLShaderProgram& shader, const QMatrix4x4& mvp);

    // 绘制世界坐标轴
    // @param shader 通用绘制 Shader
    // @param mvp 当前视图使用的模型视图投影矩阵
    // @param axesSwapped 是否以虚线方式显示 Z 轴
    void renderAxis(QOpenGLShaderProgram& shader, const QMatrix4x4& mvp, bool axesSwapped);

private:
    // 网格顶点缓冲
    QOpenGLBuffer m_gridVbo{ QOpenGLBuffer::VertexBuffer };

    // 网格顶点数组对象
    QOpenGLVertexArrayObject m_gridVao;

    // 网格顶点数量
    int m_gridVertexCount = 0;

    // 坐标轴顶点缓冲
    QOpenGLBuffer m_axisVbo{ QOpenGLBuffer::VertexBuffer };

    // 坐标轴顶点数组对象
    QOpenGLVertexArrayObject m_axisVao;

    // XY 轴实线顶点数量
    int m_axisXyVertexCount = 4;

    // Z 轴实线起始偏移
    int m_axisZSolidOffset = 4;

    // Z 轴实线顶点数量
    int m_axisZSolidVertexCount = 2;

    // Z 轴虚线起始偏移
    int m_axisZDashedOffset = 0;

    // Z 轴虚线顶点数量
    int m_axisZDashedVertexCount = 0;
};
