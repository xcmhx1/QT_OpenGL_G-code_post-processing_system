// CadOpenGLState 实现文件
// 实现 CadOpenGLState 模块，对应头文件中声明的主要行为和协作流程。
// OpenGL 状态模块，负责封装渲染阶段常用的状态切换和恢复逻辑。
#include "pch.h"

#include "CadOpenGLState.h"

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
}

// 初始化默认 OpenGL 状态
void CadOpenGLState::initializeDefaults()
{
    QOpenGLFunctions* functions = currentFunctions();

    if (functions == nullptr)
    {
        return;
    }

    // 统一启用深度、混合与点大小控制，保持整个渲染路径的默认行为一致
    functions->glEnable(GL_DEPTH_TEST);
    functions->glEnable(GL_BLEND);
    functions->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    functions->glEnable(GL_PROGRAM_POINT_SIZE);
    functions->glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
}

// 为每一帧绘制准备视口与缓冲清理
// @param framebufferWidth 当前帧缓冲宽度
// @param framebufferHeight 当前帧缓冲高度
void CadOpenGLState::prepareFrame(int framebufferWidth, int framebufferHeight)
{
    QOpenGLFunctions* functions = currentFunctions();

    if (functions == nullptr)
    {
        return;
    }

    // 每帧开始前刷新视口并清空颜色/深度缓冲
    functions->glViewport(0, 0, framebufferWidth, framebufferHeight);
    functions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}
