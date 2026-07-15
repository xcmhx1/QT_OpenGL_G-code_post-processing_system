// CadCrosshairBuilder 实现文件
// 实现 CadCrosshairBuilder 模块，对应头文件中声明的主要行为和协作流程。
// 十字光标构建模块，负责生成视图中的辅助准星和拾取框预览图元。

#include "platform/pch.h"

#include "cad/view/interaction/CadCrosshairBuilder.h"

#include "cad/view/interaction/CadInteractionConstants.h"

#include <algorithm>

void CadCrosshairBuilder::renderCrosshair
(
    QPainter& painter,
    int widgetWidth,
    int widgetHeight,
    const QPoint& cursorScreenPos,
    bool visible,
    bool orbiting,
    const QColor& color
)
{
    if (!visible || widgetWidth <= 0 || widgetHeight <= 0 || orbiting)
    {
        return;
    }

    const QPoint cursorPoint
    (
        std::clamp(cursorScreenPos.x(), 0, std::max(0, widgetWidth - 1)),
        std::clamp(cursorScreenPos.y(), 0, std::max(0, widgetHeight - 1))
    );
    const int boxHalfSize = static_cast<int>(CadInteractionConstants::kPickBoxHalfSizePixels);

    QPen pen(color, 1.0);
    pen.setCosmetic(true);
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);
    painter.drawRect
    (
        QRect
        (
            cursorPoint.x() - boxHalfSize,
            cursorPoint.y() - boxHalfSize,
            boxHalfSize * 2,
            boxHalfSize * 2
        )
    );
    painter.drawLine(0, cursorPoint.y(), widgetWidth - 1, cursorPoint.y());
    painter.drawLine(cursorPoint.x(), 0, cursorPoint.x(), widgetHeight - 1);
}
