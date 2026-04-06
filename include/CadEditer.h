#pragma once

#include <memory>
#include <vector>

#include <QColor>
#include <QVector3D>

class CadDocument;
class CadItem;
class DRW_Entity;
class DrawStateMachine;

// 文档编辑器：
// - 接收 Controller 发起的编辑/绘图命令
// - 依据 DrawStateMachine 当前子状态完成图元创建
// - 通过命令栈实现 undo / redo
class CadEditer
{
public:
    CadEditer() = default;
    ~CadEditer();

    void setDocument(CadDocument* document);
    void clearHistory();
    void cancelTransientCommand();

    bool canUndo() const;
    bool canRedo() const;

    bool undo();
    bool redo();

    bool handleLeftPress(const DrawStateMachine& previousState, DrawStateMachine& currentState, const QVector3D& worldPos);
    bool finishActivePolyline(DrawStateMachine& drawState, bool closePolyline);

    bool beginMove(DrawStateMachine& drawState, CadItem* item);
    bool deleteEntity(CadItem* item);
    bool changeEntityColor(CadItem* item, const QColor& color, int colorIndex = -1);

    class EditCommand
    {
    public:
        virtual ~EditCommand() = default;
        virtual bool execute() = 0;
        virtual bool undo() = 0;
    };

private:
    bool executeCommand(std::unique_ptr<EditCommand> command);

    bool handlePointDrawing(const DrawStateMachine& previousState, DrawStateMachine& currentState, const QVector3D& worldPos);
    bool handleLineDrawing(const DrawStateMachine& previousState, DrawStateMachine& currentState, const QVector3D& worldPos);
    bool handleCircleDrawing(const DrawStateMachine& previousState, DrawStateMachine& currentState, const QVector3D& worldPos);
    bool handleArcDrawing(const DrawStateMachine& previousState, DrawStateMachine& currentState, const QVector3D& worldPos);
    bool handleEllipseDrawing(const DrawStateMachine& previousState, DrawStateMachine& currentState, const QVector3D& worldPos);
    bool handlePolylineDrawing(const DrawStateMachine& previousState, DrawStateMachine& currentState, const QVector3D& worldPos, bool lightweight);
    bool handleMoveEditing(const DrawStateMachine& previousState, DrawStateMachine& currentState, const QVector3D& worldPos);

    bool addEntity(std::unique_ptr<DRW_Entity> entity);

private:
    CadDocument* m_document = nullptr;
    CadItem* m_moveTarget = nullptr;
    std::vector<std::unique_ptr<EditCommand>> m_undoStack;
    std::vector<std::unique_ptr<EditCommand>> m_redoStack;
};
