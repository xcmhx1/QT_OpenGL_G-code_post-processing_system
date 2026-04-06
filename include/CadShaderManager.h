#pragma once

#include <memory>

#include <QtGlobal>
#include <QOpenGLShaderProgram>

class CadShaderManager
{
public:
    void initialize();
    void destroy();
    bool isInitialized() const;

    QOpenGLShaderProgram& generalShader() const;

private:
    std::unique_ptr<QOpenGLShaderProgram> m_generalShader;
};
