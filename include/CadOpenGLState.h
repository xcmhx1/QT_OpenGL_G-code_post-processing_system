// 声明 CadOpenGLState 模块，对外暴露当前组件的核心类型、接口和协作边界。
// OpenGL 状态模块，负责封装渲染阶段常用的状态切换和恢复逻辑。
#pragma once

class CadOpenGLState
{
public:
    static void initializeDefaults();
    static void prepareFrame(int framebufferWidth, int framebufferHeight);
};
