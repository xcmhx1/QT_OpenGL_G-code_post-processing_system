// 声明 CadController 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 输入控制模块，负责解释键盘、鼠标、滚轮事件并驱动绘图/编辑命令。
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
