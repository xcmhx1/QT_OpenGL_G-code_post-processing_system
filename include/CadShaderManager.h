// 声明 CadShaderManager 模块，对外暴露当前组件的核心类型、接口和协作边界。
// Shader 管理模块，负责创建、持有并提供绘制流程所需的着色器程序。
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
