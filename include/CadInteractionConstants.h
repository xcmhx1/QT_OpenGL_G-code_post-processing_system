// 声明 CadInteractionConstants 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 交互常量模块，集中维护拾取、十字光标和视图交互使用的固定参数。
#pragma once

namespace CadInteractionConstants
{
    constexpr float kPickBoxHalfSizePixels = 10.0f;
    constexpr float kCrosshairHalfLengthWorld = 100000.0f;
}
