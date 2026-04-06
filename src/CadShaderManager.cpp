#include "pch.h"

#include "CadShaderManager.h"

void CadShaderManager::initialize()
{
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

    m_generalShader = std::make_unique<QOpenGLShaderProgram>();
    m_generalShader->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    m_generalShader->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    m_generalShader->link();
}

void CadShaderManager::destroy()
{
    m_generalShader.reset();
}

bool CadShaderManager::isInitialized() const
{
    return m_generalShader != nullptr;
}

QOpenGLShaderProgram& CadShaderManager::generalShader() const
{
    Q_ASSERT(m_generalShader != nullptr);
    return *m_generalShader;
}
