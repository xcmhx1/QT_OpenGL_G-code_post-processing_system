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
    void initializeGridBuffer();
    void initializeAxisBuffer();
    void destroy();

    void renderGrid(QOpenGLShaderProgram& shader, const QMatrix4x4& mvp);
    void renderAxis(QOpenGLShaderProgram& shader, const QMatrix4x4& mvp, bool axesSwapped);

private:
    QOpenGLBuffer m_gridVbo{ QOpenGLBuffer::VertexBuffer };
    QOpenGLVertexArrayObject m_gridVao;
    int m_gridVertexCount = 0;

    QOpenGLBuffer m_axisVbo{ QOpenGLBuffer::VertexBuffer };
    QOpenGLVertexArrayObject m_axisVao;
    int m_axisXyVertexCount = 4;
    int m_axisZSolidOffset = 4;
    int m_axisZSolidVertexCount = 2;
    int m_axisZDashedOffset = 0;
    int m_axisZDashedVertexCount = 0;
};
