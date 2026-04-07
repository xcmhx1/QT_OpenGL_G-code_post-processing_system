// CadOverlayRenderer 实现文件
// 实现 CadOverlayRenderer 模块，对应头文件中声明的主要行为和协作流程。
// 叠加层渲染模块，负责命令预览、十字光标等 overlay 图元的绘制。
#include "pch.h"

#include "CadOverlayRenderer.h"

#include <QOpenGLContext>
#include <QOpenGLFunctions>

namespace
{
    // 获取当前 OpenGL 上下文导出的函数表
    // @return 可用的 OpenGL 函数表指针，无上下文时返回 nullptr
    QOpenGLFunctions* currentFunctions()
    {
        if (QOpenGLContext* context = QOpenGLContext::currentContext())
        {
            return context->functions();
        }

        return nullptr;
    }

    // 绑定位置属性为三维浮点顶点输入
    void bindVec3VertexAttribute()
    {
        if (QOpenGLFunctions* functions = currentFunctions())
        {
            functions->glEnableVertexAttribArray(0);
            functions->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);
        }
    }
}

// 初始化轨道中心标记缓冲
void CadOverlayRenderer::initializeOrbitMarkerBuffer()
{
    const QVector3D initialPoint(0.0f, 0.0f, 0.0f);

    m_orbitMarkerVao.create();
    m_orbitMarkerVao.bind();

    m_orbitMarkerVbo.create();
    m_orbitMarkerVbo.bind();
    m_orbitMarkerVbo.allocate(&initialPoint, static_cast<int>(sizeof(QVector3D)));

    bindVec3VertexAttribute();

    m_orbitMarkerVbo.release();
    m_orbitMarkerVao.release();
}

// 初始化 transient 图元缓冲
void CadOverlayRenderer::initializeTransientBuffer()
{
    const QVector3D initialPoint(0.0f, 0.0f, 0.0f);

    m_transientVao.create();
    m_transientVao.bind();

    m_transientVbo.create();
    m_transientVbo.bind();
    m_transientVbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    m_transientVbo.allocate(&initialPoint, static_cast<int>(sizeof(QVector3D)));

    bindVec3VertexAttribute();

    m_transientVbo.release();
    m_transientVao.release();
}

// 销毁叠加层相关 GPU 资源
void CadOverlayRenderer::destroy()
{
    m_orbitMarkerVao.destroy();
    m_orbitMarkerVbo.destroy();
    m_transientVao.destroy();
    m_transientVbo.destroy();
}

// 查询轨道中心标记缓冲是否可用
// @return 如果缓冲已创建返回 true，否则返回 false
bool CadOverlayRenderer::orbitMarkerReady() const
{
    return m_orbitMarkerVao.isCreated() && m_orbitMarkerVbo.isCreated();
}

// 查询 transient 图元缓冲是否可用
// @return 如果缓冲已创建返回 true，否则返回 false
bool CadOverlayRenderer::transientReady() const
{
    return m_transientVao.isCreated() && m_transientVbo.isCreated();
}

// 绘制轨道观察中心标记
// @param shader 通用绘制 Shader
// @param mvp 当前视图使用的模型视图投影矩阵
// @param orbitCenter 轨道观察中心点
// @param visible 标记是否可见
void CadOverlayRenderer::renderOrbitMarker
(
    QOpenGLShaderProgram& shader,
    const QMatrix4x4& mvp,
    const QVector3D& orbitCenter,
    bool visible
)
{
    // 标记不可见或缓冲未就绪时跳过绘制
    if (!visible || !orbitMarkerReady())
    {
        return;
    }

    QOpenGLFunctions* functions = currentFunctions();

    if (functions == nullptr)
    {
        return;
    }

    // 每次绘制前更新当前轨道中心位置
    m_orbitMarkerVbo.bind();
    m_orbitMarkerVbo.write(0, &orbitCenter, static_cast<int>(sizeof(QVector3D)));
    m_orbitMarkerVbo.release();

    shader.bind();
    shader.setUniformValue("uMvp", mvp);
    shader.setUniformValue("uColor", QVector3D(0.10f, 0.95f, 0.25f));
    shader.setUniformValue("uPointSize", 14.0f);
    shader.setUniformValue("uRoundPoint", 1);

    functions->glDisable(GL_DEPTH_TEST);

    m_orbitMarkerVao.bind();
    functions->glDrawArrays(GL_POINTS, 0, 1);
    m_orbitMarkerVao.release();

    functions->glEnable(GL_DEPTH_TEST);
    shader.release();
}

// 绘制 transient 叠加图元
// @param shader 通用绘制 Shader
// @param mvp 当前视图使用的模型视图投影矩阵
// @param commandPrimitives 命令预览图元集合
// @param crosshairPrimitives 十字光标与拾取框图元集合
void CadOverlayRenderer::renderTransientPrimitives
(
    QOpenGLShaderProgram& shader,
    const QMatrix4x4& mvp,
    const std::vector<TransientPrimitive>& commandPrimitives,
    const std::vector<TransientPrimitive>& crosshairPrimitives
)
{
    // 无可用缓冲或没有任何 overlay 图元时不进入绘制流程
    if (!transientReady() || (commandPrimitives.empty() && crosshairPrimitives.empty()))
    {
        return;
    }

    QOpenGLFunctions* functions = currentFunctions();

    if (functions == nullptr)
    {
        return;
    }

    // transient 图元共用一套 VAO/VBO，每个 primitive 逐个上传后立刻绘制
    shader.bind();
    shader.setUniformValue("uMvp", mvp);
    m_transientVao.bind();

    auto drawPrimitives = [this, &shader, functions](const std::vector<TransientPrimitive>& primitives)
    {
        for (const TransientPrimitive& primitive : primitives)
        {
            if (primitive.vertices.isEmpty())
            {
                continue;
            }

            m_transientVbo.bind();
            m_transientVbo.allocate
            (
                primitive.vertices.constData(),
                primitive.vertices.size() * static_cast<int>(sizeof(QVector3D))
            );

            shader.setUniformValue("uColor", primitive.color);
            shader.setUniformValue("uPointSize", primitive.pointSize);
            shader.setUniformValue("uRoundPoint", primitive.roundPoint ? 1 : 0);

            functions->glDrawArrays(primitive.primitiveType, 0, primitive.vertices.size());
            m_transientVbo.release();
        }
    };

    if (!commandPrimitives.empty())
    {
        functions->glDisable(GL_DEPTH_TEST);
        functions->glLineWidth(2.0f);
        // 命令预览强调可读性，因此固定使用更粗的线宽并关闭深度测试
        drawPrimitives(commandPrimitives);
    }

    if (!crosshairPrimitives.empty())
    {
        functions->glEnable(GL_DEPTH_TEST);
        functions->glLineWidth(1.0f);
        // 十字光标需要与场景深度关系保持一致，恢复深度测试后绘制
        drawPrimitives(crosshairPrimitives);
    }

    m_transientVao.release();
    functions->glLineWidth(1.0f);
    functions->glEnable(GL_DEPTH_TEST);
    shader.release();
}
