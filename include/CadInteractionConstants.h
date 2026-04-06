// 声明 CadInteractionConstants 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 交互常量模块，集中维护拾取、十字光标和视图交互使用的固定参数。
#pragma once

namespace CadInteractionConstants
{
    constexpr float kPickBoxHalfSizePixels = 10.0f;
    // 十字线当前按世界坐标长度绘制，过大会让 overlay 在拖动时显得发涩。
    constexpr float kCrosshairHalfLengthWorld = 5000.0f;
}
