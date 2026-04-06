#pragma once

#include <QColor>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QString>
#include <QVector3D>
#include <QWheelEvent>

#include "DrawStateMachine.h"

class CadViewer;
class CadEditer;

// 解释用户输入
class CadController
{
public:
    void setViewer(CadViewer* viewer);
    void setEditer(CadEditer* editer);
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
    QString currentPrompt() const;
    QString currentCommandName() const;

private:
    void resetSubModes();
    void preparePrimitiveSubMode();
    void handleLeftPressInCommand(const QVector3D& worldPos);
    bool isPolylineCommandActive() const;
    bool setPolylineInputMode(bool useArc);
    QVector3D currentWorldPos(const QPoint& screenPos) const;

private:
    CadViewer* m_viewer = nullptr;
    CadEditer* m_editer = nullptr;
    DrawStateMachine m_drawState;
};
