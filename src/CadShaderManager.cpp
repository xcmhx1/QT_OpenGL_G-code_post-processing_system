// CadShaderManager 实现文件
// 实现 CadShaderManager 模块，对应头文件中声明的主要行为和协作流程。
// Shader 管理模块，负责创建、持有并提供绘制流程所需的着色器程序。
#include "pch.h"

#include "CadShaderManager.h"

// 初始化通用 Shader 程序
void CadShaderManager::initialize()
{
    // 通用顶点着色器：完成 MVP 变换并支持点大小控制
    static constexpr const char* vertexShaderSource = R"(
        #version 450 core
        layout(location = 0) in vec3 aPosition;
        uniform mat4 uMvp;
        uniform float uPointSize;
        void main()
        {
            gl_Position = uMvp * vec4(aPosition, 1.0);
            gl_PointSize = uPointSize;
        }
    )";

    // 通用片元着色器：统一处理纯色输出，并可按需裁剪成圆点
    static constexpr const char* fragmentShaderSource = R"(
        #version 450 core
        uniform vec3 uColor;
        uniform int uRoundPoint;
        out vec4 fragColor;
        void main()
        {
            if (uRoundPoint != 0)
            {
                vec2 pointCoord = gl_PointCoord * 2.0 - vec2(1.0, 1.0);
                if (dot(pointCoord, pointCoord) > 1.0)
                {
                    discard;
                }
            }

            fragColor = vec4(uColor, 1.0);
        }
    )";

    // 创建并链接当前渲染路径共用的 Shader 程序
    m_generalShader = std::make_unique<QOpenGLShaderProgram>();
    m_generalShader->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    m_generalShader->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    m_generalShader->link();
}

// 销毁已创建的 Shader 程序
void CadShaderManager::destroy()
{
    m_generalShader.reset();
}

// 查询 Shader 管理器是否已经初始化
// @return 如果通用 Shader 已创建返回 true，否则返回 false
bool CadShaderManager::isInitialized() const
{
    return m_generalShader != nullptr;
}

// 获取通用绘制 Shader
// @return 通用着色器程序引用
QOpenGLShaderProgram& CadShaderManager::generalShader() const
{
    Q_ASSERT(m_generalShader != nullptr);
    return *m_generalShader;
}
