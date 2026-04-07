// CadCrosshairBuilder 实现文件
// 实现 CadCrosshairBuilder 模块，对应头文件中声明的主要行为和协作流程。
// 十字光标构建模块，负责生成视图中的辅助准星和拾取框预览图元。

#include "pch.h"

#include "CadCrosshairBuilder.h"

// CAD 模块内部依赖
#include "CadInteractionConstants.h"
#include "CadViewTransform.h"

// 标准库
#include <algorithm>

// 匿名命名空间，存放局部常量
namespace
{
    // 十字准线颜色（青色）
    const QVector3D kCrosshairColor(0.38f, 0.88f, 1.0f);
}

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
std::vector<TransientPrimitive> CadCrosshairBuilder::buildCrosshairPrimitives
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
)
{
    // 返回的临时图元列表
    std::vector<TransientPrimitive> primitives;

    // 如果不可见、部件尺寸无效或正在轨道旋转，则返回空列表
    if (!visible || widgetWidth <= 0 || widgetHeight <= 0 || orbiting)
    {
        return primitives;
    }

    // 将光标坐标限制在部件范围内
    const QPoint cursorPoint
    (
        std::clamp(cursorScreenPos.x(), 0, std::max(0, widgetWidth - 1)),
        std::clamp(cursorScreenPos.y(), 0, std::max(0, widgetHeight - 1))
    );

    // 将屏幕坐标转换为指定平面上的世界坐标
    const QVector3D worldCenter = CadViewTransform::screenToPlane(camera, viewportWidth, viewportHeight, cursorPoint, planeZ);

    // 构建水平线图元
    TransientPrimitive horizontalLine;
    horizontalLine.primitiveType = GL_LINES;  // 使用线段绘制
    horizontalLine.color = kCrosshairColor;   // 设置颜色
    // 设置水平线段的两个端点
    horizontalLine.vertices =
    {
        worldCenter + QVector3D(-crosshairHalfLengthWorld, 0.0f, 0.0f),  // 左端点
        worldCenter + QVector3D(crosshairHalfLengthWorld, 0.0f, 0.0f)    // 右端点
    };
    primitives.push_back(std::move(horizontalLine));

    // 构建垂直线图元
    TransientPrimitive verticalLine;
    verticalLine.primitiveType = GL_LINES;  // 使用线段绘制
    verticalLine.color = kCrosshairColor;   // 设置颜色
    // 设置垂直线段的两个端点
    verticalLine.vertices =
    {
        worldCenter + QVector3D(0.0f, -crosshairHalfLengthWorld, 0.0f),  // 下端点
        worldCenter + QVector3D(0.0f, crosshairHalfLengthWorld, 0.0f)    // 上端点
    };
    primitives.push_back(std::move(verticalLine));

    // 构建拾取框图元
    TransientPrimitive pickBox;
    pickBox.primitiveType = GL_LINE_STRIP;  // 使用线段条带绘制（首尾相连）
    pickBox.color = kCrosshairColor;        // 设置颜色
    // 设置拾取框的五个顶点（起点、顺时针、最后回到起点闭合）
    pickBox.vertices =
    {
        worldCenter + QVector3D(-boxHalfSizeWorld, -boxHalfSizeWorld, 0.0f),  // 左下角
        worldCenter + QVector3D(boxHalfSizeWorld, -boxHalfSizeWorld, 0.0f),   // 右下角
        worldCenter + QVector3D(boxHalfSizeWorld, boxHalfSizeWorld, 0.0f),    // 右上角
        worldCenter + QVector3D(-boxHalfSizeWorld, boxHalfSizeWorld, 0.0f),   // 左上角
        worldCenter + QVector3D(-boxHalfSizeWorld, -boxHalfSizeWorld, 0.0f)   // 回到左下角
    };
    primitives.push_back(std::move(pickBox));

    return primitives;
}