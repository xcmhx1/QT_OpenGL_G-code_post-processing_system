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
