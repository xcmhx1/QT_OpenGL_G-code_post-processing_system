// CadInteractionConstants 头文件
// 声明 CadInteractionConstants 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 交互常量模块，集中维护拾取、十字光标和视图交互使用的固定参数。

#pragma once

// CadInteractionConstants 命名空间：
// 包含CAD交互系统中使用的所有常量，这些常量在整个应用程序中共享
namespace CadInteractionConstants
{
    // 拾取框半宽（像素）
    // 用于定义实体拾取时鼠标点击的敏感区域大小
    // 拾取框的实际尺寸为 2 * kPickBoxHalfSizePixels
    constexpr float kPickBoxHalfSizePixels = 10.0f;

    // 十字准线半长（世界单位）
    // 注意：十字线当前按世界坐标长度绘制，过大会让 overlay 在拖动时显得发涩
    // 这个值定义了十字准线在水平和垂直方向上的长度的一半
    constexpr float kCrosshairHalfLengthWorld = 500.0f;
}