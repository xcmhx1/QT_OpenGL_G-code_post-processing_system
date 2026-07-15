// CadShaderManager 头文件
// 声明 CadShaderManager 模块，对外暴露当前组件的核心类型、接口和协作边界。
// Shader 管理模块，负责创建、持有并提供绘制流程所需的着色器程序。
#pragma once

#include <memory>

#include <QtGlobal>
#include <QOpenGLShaderProgram>

class CadShaderManager
{
public:
    // 初始化通用 Shader 程序
    void initialize();

    // 销毁已创建的 Shader 程序
    void destroy();

    // 查询 Shader 管理器是否已经初始化
    // @return 如果通用 Shader 已创建返回 true，否则返回 false
    bool isInitialized() const;

    // 获取通用绘制 Shader
    // @return 通用着色器程序引用
    QOpenGLShaderProgram& generalShader() const;

private:
    // 通用线框/点绘制 Shader
    std::unique_ptr<QOpenGLShaderProgram> m_generalShader;
};
