#pragma once

#include <QColor>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QVector3D>
#include <QWheelEvent>

#include "DrawStateMachine.h"

class CadViewer;

// 解释用户输入
class CadController
{
public:
    void setViewer(CadViewer* viewer);
    void reset();

    void beginDrawing(DrawType primitiveKind, const QColor& color = QColor(255, 255, 255));
    void cancelDrawing();

    bool handleMousePress(QMouseEvent* event);
    bool handleMouseMove(QMouseEvent* event);
    bool handleMouseRelease(QMouseEvent* event);
    bool handleWheel(QWheelEvent* event);
    bool handleKeyPress(QKeyEvent* event);

    DrawStateMachine& drawState();
    const DrawStateMachine& drawState() const;

private:
    void resetSubModes();
    void preparePrimitiveSubMode();
    void handleLeftPressInDrawing(const QVector3D& worldPos);
    QVector3D currentWorldPos(const QPoint& screenPos) const;

private:
    CadViewer* m_viewer = nullptr;
    DrawStateMachine m_drawState;
};
