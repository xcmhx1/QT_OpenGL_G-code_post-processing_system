// CadCrosshairBuilder 头文件
// 声明 CadCrosshairBuilder 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 十字光标构建模块，负责生成视图中的辅助准星和拾取框预览图元。

#pragma once

// 标准库
#include <vector>

// Qt 核心模块
#include <QPoint>

// CAD 模块内部依赖
#include "CadCamera.h"
#include "CadRenderTypes.h"

// 十字光标构建器类：
// 静态工具类，用于构建CAD视图中的十字准线、拾取框等临时渲染图元。
// 这些图元用于辅助用户进行精确定位和实体选择。
class CadCrosshairBuilder
{
public:
    // 构建十字准线图元
    // 根据当前视图状态、相机参数和光标位置，生成十字准线和拾取框的渲染图元
    // @param camera 当前轨道相机对象
    // @param viewportWidth 视口宽度（像素）
    // @param viewportHeight 视口高度（像素）
    // @param widgetWidth 部件宽度（逻辑像素）
    // @param widgetHeight 部件高度（逻辑像素）
    // @param cursorScreenPos 当前光标屏幕坐标
    // @param visible 十字准线是否可见
    // @param orbiting 是否正在轨道旋转交互中
    // @param planeZ 十字准线平面Z坐标（世界坐标）
    // @param boxHalfSizeWorld 拾取框半边长（世界单位）
    // @param crosshairHalfLengthWorld 十字准线半长（世界单位）
    // @return 临时图元向量，包含十字准线和拾取框的渲染数据
    static std::vector<TransientPrimitive> buildCrosshairPrimitives
    (
        const OrbitalCamera& camera,
        int viewportWidth,
        int viewportHeight,
        int widgetWidth,
        int widgetHeight,
        const QPoint& cursorScreenPos,
        bool visible,
        bool orbiting,
        float planeZ,
        float boxHalfSizeWorld,
        float crosshairHalfLengthWorld
    );
};