// CadCrosshairBuilder 头文件
// 声明 CadCrosshairBuilder 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 十字光标构建模块，负责生成视图中的辅助准星和拾取框预览图元。

#pragma once

#include <QColor>
#include <QPainter>
#include <QPoint>

// 十字光标构建器类：
// 静态工具类，用于构建CAD视图中的十字准线、拾取框等临时渲染图元。
// 这些图元用于辅助用户进行精确定位和实体选择。
class CadCrosshairBuilder
{
public:
    // 在屏幕空间绘制十字线，使其不受视角、缩放和吸附位置影响。
    static void renderCrosshair
    (
        QPainter& painter,
        int widgetWidth,
        int widgetHeight,
        const QPoint& cursorScreenPos,
        bool visible,
        bool orbiting,
        const QColor& color
    );
};
