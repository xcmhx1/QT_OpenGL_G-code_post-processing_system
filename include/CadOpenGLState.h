// CadOpenGLState 头文件
// 声明 CadOpenGLState 模块，对外暴露当前组件的核心类型、接口和协作边界。
// OpenGL 状态模块，负责封装渲染阶段常用的状态切换和恢复逻辑。
#pragma once

class CadOpenGLState
{
public:
    // 初始化默认 OpenGL 状态
    static void initializeDefaults();

    // 为每一帧绘制准备视口与缓冲清理
    // @param framebufferWidth 当前帧缓冲宽度
    // @param framebufferHeight 当前帧缓冲高度
    static void prepareFrame(int framebufferWidth, int framebufferHeight);
};
