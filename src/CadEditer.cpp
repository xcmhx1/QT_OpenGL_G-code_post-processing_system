// CadEditer 实现文件
// 实现 CadEditer 模块，对应头文件中声明的主要行为和协作流程。
// 编辑器模块，负责绘图创建、实体修改以及 Undo/Redo 命令栈管理。
#include "pch.h"

#include "CadEditer.h"

#include "CadDocument.h"
#include "CadItem.h"
#include "DrawStateMachine.h"

// 析构编辑器对象
CadEditer::~CadEditer() = default;

// 绑定当前编辑目标文档
// @param document 文档对象指针
void CadEditer::setDocument(CadDocument* document)
{
    if (m_document == document)
    {
        return;
    }

    clearHistory();
    m_document = document;
}

// 清空 Undo / Redo 历史
void CadEditer::clearHistory()
{
    m_undoStack.clear();
    m_redoStack.clear();
    cancelTransientCommand();
}

// 取消当前 transient 编辑命令
void CadEditer::cancelTransientCommand()
{
    m_moveTarget = nullptr;
    m_moveTargets.clear();
    m_gripTarget = nullptr;
    m_gripPointIndex = -1;
}

void CadEditer::setProcessState(cadcam::process::DocumentProcessState* processState)
{
    m_processState = processState;
}

// 查询是否可以撤销
// @return 如果撤销栈非空返回 true，否则返回 false
bool CadEditer::canUndo() const
{
    return !m_undoStack.empty();
}

// 查询是否可以重做
// @return 如果重做栈非空返回 true，否则返回 false
bool CadEditer::canRedo() const
{
    return !m_redoStack.empty();
}

// 执行撤销
// @return 如果撤销成功返回 true，否则返回 false
bool CadEditer::undo()
{
    // 从 Undo 栈弹出命令，撤销成功后转移到 Redo 栈
    if (m_undoStack.empty())
    {
        return false;
    }

    std::unique_ptr<EditCommand> command = std::move(m_undoStack.back());
    m_undoStack.pop_back();
    auto contentBatch = m_document->beginContentChangeBatch();

    if (!command->undo())
    {
        return false;
    }

    m_redoStack.push_back(std::move(command));
    return true;
}

// 执行重做
// @return 如果重做成功返回 true，否则返回 false
bool CadEditer::redo()
{
    // 从 Redo 栈弹出命令，重做成功后重新压回 Undo 栈
    if (m_redoStack.empty())
    {
        return false;
    }

    std::unique_ptr<EditCommand> command = std::move(m_redoStack.back());
    m_redoStack.pop_back();
    auto contentBatch = m_document->beginContentChangeBatch();

    if (!command->execute())
    {
        return false;
    }

    m_undoStack.push_back(std::move(command));
    return true;
}

// 处理左键点击驱动的绘图或编辑逻辑
bool CadEditer::handleLeftPress
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    // Move 编辑优先于绘图命令处理
    if (m_document == nullptr)
    {
        return false;
    }

    if (currentState.editType == EditType::Move || previousState.editType == EditType::Move)
    {
        return handleMoveEditing(previousState, currentState, worldPos);
    }

    if (currentState.editType == EditType::GripEdit || previousState.editType == EditType::GripEdit)
    {
        return handleGripEditing(previousState, currentState, worldPos);
    }

    if (!currentState.isDrawing && !previousState.isDrawing)
    {
        return false;
    }

    // 其它绘图命令按前一时刻的 drawType 分发，确保状态机切换前后语义一致
    switch (previousState.drawType)
    {
    case DrawType::Point:
        return handlePointDrawing(previousState, currentState, worldPos);
    case DrawType::Line:
        return handleLineDrawing(previousState, currentState, worldPos);
    case DrawType::Xline:
        return handleXlineDrawing(previousState, currentState, worldPos);
    case DrawType::Rectangle:
        return handleRectangleDrawing(previousState, currentState, worldPos);
    case DrawType::Polygon:
        return handlePolygonDrawing(previousState, currentState, worldPos);
    case DrawType::Circle:
        return handleCircleDrawing(previousState, currentState, worldPos);
    case DrawType::Arc:
        return handleArcDrawing(previousState, currentState, worldPos);
    case DrawType::Ellipse:
        return handleEllipseDrawing(previousState, currentState, worldPos);
    case DrawType::Polyline:
        return handlePolylineDrawing(previousState, currentState, worldPos, false);
    case DrawType::LWPolyline:
        return handlePolylineDrawing(previousState, currentState, worldPos, true);
    default:
        break;
    }

    return false;
}

