#pragma once

#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QVector>
#include <QVector3D>
#include <QtGlobal>

using EntityId = quintptr;

// 单个实体对应的一组 GPU 资源。
struct EntityGpuBuffer
{
    // 顶点缓冲。
    QOpenGLBuffer vbo{ QOpenGLBuffer::VertexBuffer };

    // 顶点数组对象，封装顶点属性绑定状态。
    QOpenGLVertexArrayObject vao;

    // 顶点数量。
    int vertexCount = 0;

    // OpenGL 图元类型，如 GL_LINES / GL_LINE_STRIP / GL_POINTS。
    GLenum primitiveType = GL_LINE_STRIP;

    // 实体绘制颜色（RGB，0~1）。
    QVector3D color = { 1.0f, 1.0f, 1.0f };
};

struct TransientPrimitive
{
    QVector<QVector3D> vertices;
    GLenum primitiveType = GL_LINE_STRIP;
    QVector3D color = { 0.25f, 0.85f, 1.0f };
    float pointSize = 1.0f;
    bool roundPoint = false;
};
