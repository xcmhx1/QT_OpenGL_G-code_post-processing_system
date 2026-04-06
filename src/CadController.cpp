// 实现 CadController 模块，对应头文件中声明的主要行为和协作流程。
// 输入控制模块，负责解释键盘、鼠标、滚轮事件并驱动绘图/编辑命令。
#include "pch.h"

#include "CadController.h"

#include <QColorDialog>

#include "CadEditer.h"
#include "CadItem.h"
#include "CadViewer.h"

namespace
{
    QVector3D flattenToDrawingPlane(const QVector3D& point)
    {
        return QVector3D(point.x(), point.y(), 0.0f);
    }

    QString drawTypeName(DrawType drawType)
    {
        switch (drawType)
        {
        case DrawType::Point:
            return QStringLiteral("点");
        case DrawType::Line:
            return QStringLiteral("直线");
        case DrawType::Circle:
            return QStringLiteral("圆");
        case DrawType::Arc:
            return QStringLiteral("圆弧");
        case DrawType::Ellipse:
            return QStringLiteral("椭圆");
        case DrawType::Polyline:
            return QStringLiteral("多段线");
        case DrawType::LWPolyline:
            return QStringLiteral("轻量多段线");
        default:
            return QStringLiteral("空闲");
        }
    }
}

void CadController::setViewer(CadViewer* viewer)
{
    m_viewer = viewer;
}

void CadController::setEditer(CadEditer* editer)
{
    m_editer = editer;
}

void CadController::reset()
{
    if (m_editer != nullptr)
    {
        m_editer->cancelTransientCommand();
    }

    m_drawState.reset();

    if (m_viewer != nullptr)
    {
        m_viewer->refreshCommandPrompt();
    }
}

void CadController::beginDrawing(DrawType drawType, const QColor& color)
{
    if (m_editer != nullptr)
    {
        m_editer->cancelTransientCommand();
    }

    m_drawState.isDrawing = true;
    m_drawState.drawType = drawType;
    m_drawState.drawingColor = color;
    m_drawState.editType = EditType::None;
    m_drawState.commandPoints.clear();
    m_drawState.commandBulges.clear();
    m_drawState.polylineArcMode = false;
    m_drawState.lwPolylineArcMode = false;
    resetSubModes();
    preparePrimitiveSubMode();

    if (m_viewer != nullptr)
    {
        m_viewer->appendCommandMessage(QStringLiteral("已进入%1命令").arg(drawTypeName(drawType)));
        m_viewer->refreshCommandPrompt();
    }
}

void CadController::cancelDrawing()
{
    const bool hadActiveCommand = m_drawState.hasActiveCommand();

    if (m_editer != nullptr)
    {
        m_editer->cancelTransientCommand();
    }

    m_drawState.isDrawing = false;
    m_drawState.drawType = DrawType::None;
    m_drawState.editType = EditType::None;
    m_drawState.commandPoints.clear();
    m_drawState.commandBulges.clear();
    m_drawState.polylineArcMode = false;
    m_drawState.lwPolylineArcMode = false;
    resetSubModes();

    if (m_viewer != nullptr)
    {
        if (hadActiveCommand)
        {
            m_viewer->appendCommandMessage(QStringLiteral("命令已取消"));
        }

        m_viewer->refreshCommandPrompt();
    }
}

bool CadController::handleMousePress(QMouseEvent* event)
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    const QVector3D worldPos = currentWorldPos(event->pos());

    m_drawState.pressScreenPos = event->pos();
    m_drawState.lastScreenPos = event->pos();
    m_drawState.currentScreenPos = event->pos();
    m_drawState.activeButton = event->button();
    m_drawState.pressedButtons = event->buttons();
    m_drawState.keyboardModifiers = event->modifiers();
    m_drawState.lastPos = m_drawState.currentPos;
    m_drawState.currentPos = worldPos;

    if (event->button() == Qt::MiddleButton)
    {
        if ((event->modifiers() & Qt::ShiftModifier) != 0)
        {
            m_viewer->beginOrbitInteraction();
        }
        else
        {
            m_viewer->beginPanInteraction();
        }

        return true;
    }

    if (event->button() == Qt::LeftButton)
    {
        const DrawStateMachine previousState = m_drawState;
        handleLeftPressInCommand(worldPos);

        if (m_editer != nullptr && m_editer->handleLeftPress(previousState, m_drawState, worldPos))
        {
            if (m_viewer != nullptr)
            {
                if (previousState.editType == EditType::Move && m_drawState.editType == EditType::None)
                {
                    m_viewer->appendCommandMessage(QStringLiteral("移动完成"));
                }
                else if (previousState.drawType == DrawType::Point)
                {
                    m_viewer->appendCommandMessage(QStringLiteral("已创建点图元"));
                }
                else if (previousState.drawType == DrawType::Circle
                    && previousState.circleSubMode == CircleDrawSubMode::AwaitRadius
                    && m_drawState.circleSubMode == CircleDrawSubMode::AwaitCenter)
                {
                    m_viewer->appendCommandMessage(QStringLiteral("已创建圆图元"));
                }
                else if (previousState.drawType == DrawType::Arc
                    && previousState.arcSubMode == ArcDrawSubMode::AwaitEndAngle
                    && m_drawState.arcSubMode == ArcDrawSubMode::AwaitCenter)
                {
                    m_viewer->appendCommandMessage(QStringLiteral("已创建圆弧图元"));
                }
                else if (previousState.drawType == DrawType::Ellipse
                    && previousState.ellipseSubMode == EllipseDrawSubMode::AwaitMinorAxis
                    && m_drawState.ellipseSubMode == EllipseDrawSubMode::AwaitCenter)
                {
                    m_viewer->appendCommandMessage(QStringLiteral("已创建椭圆图元"));
                }
                else if (previousState.drawType == DrawType::Line
                    && previousState.lineSubMode == LineDrawSubMode::AwaitEndPoint
                    && m_drawState.lineSubMode == LineDrawSubMode::AwaitEndPoint)
                {
                    m_viewer->appendCommandMessage(QStringLiteral("已创建直线图元"));
                }

                m_viewer->refreshCommandPrompt();
            }

            return true;
        }

        m_viewer->selectEntityAt(event->pos());
        m_viewer->refreshCommandPrompt();
        return true;
    }

    return false;
}

bool CadController::handleMouseMove(QMouseEvent* event)
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    const QVector3D worldPos = currentWorldPos(event->pos());

    m_drawState.lastScreenPos = m_drawState.currentScreenPos;
    m_drawState.currentScreenPos = event->pos();
    m_drawState.pressedButtons = event->buttons();
    m_drawState.keyboardModifiers = event->modifiers();
    m_drawState.lastPos = m_drawState.currentPos;
    m_drawState.currentPos = worldPos;

    if (m_viewer->interactionMode() == ViewInteractionMode::Orbiting && m_viewer->shouldIgnoreNextOrbitDelta())
    {
        m_viewer->consumeIgnoreNextOrbitDelta();
        return true;
    }

    const QPoint delta = m_drawState.currentScreenPos - m_drawState.lastScreenPos;

    switch (m_viewer->interactionMode())
    {
    case ViewInteractionMode::Panning:
        m_viewer->updatePanInteraction(delta);
        return true;
    case ViewInteractionMode::Orbiting:
        m_viewer->updateOrbitInteraction(delta);
        return true;
    default:
        break;
    }

    return false;
}

bool CadController::handleMouseRelease(QMouseEvent* event)
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    m_drawState.lastScreenPos = m_drawState.currentScreenPos;
    m_drawState.currentScreenPos = event->pos();
    m_drawState.activeButton = event->button();
    m_drawState.pressedButtons = event->buttons();
    m_drawState.keyboardModifiers = event->modifiers();

    if (event->button() == Qt::MiddleButton)
    {
        m_viewer->endViewInteraction();
        return true;
    }

    return false;
}

bool CadController::handleWheel(QWheelEvent* event)
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    const QVector3D worldPos = currentWorldPos(event->position().toPoint());
    m_drawState.currentPos = worldPos;
    m_drawState.keyboardModifiers = event->modifiers();

    const float factor = event->angleDelta().y() > 0 ? 1.1f : (1.0f / 1.1f);
    m_viewer->zoomAtScreenPosition(event->position().toPoint(), factor);
    event->accept();
    return true;
}

bool CadController::handleKeyPress(QKeyEvent* event)
{
    m_drawState.keyboardModifiers = event->modifiers();

    if ((event->modifiers() & Qt::ControlModifier) != 0 && m_editer != nullptr)
    {
        if (event->key() == Qt::Key_Z && (event->modifiers() & Qt::ShiftModifier) != 0)
        {
            const bool handled = m_editer->redo();

            if (handled && m_viewer != nullptr)
            {
                m_viewer->appendCommandMessage(QStringLiteral("重做完成"));
                m_viewer->refreshCommandPrompt();
            }

            return handled;
        }

        if (event->key() == Qt::Key_Z)
        {
            const bool handled = m_editer->undo();

            if (handled && m_viewer != nullptr)
            {
                m_viewer->appendCommandMessage(QStringLiteral("撤销完成"));
                m_viewer->refreshCommandPrompt();
            }

            return handled;
        }

        if (event->key() == Qt::Key_Y)
        {
            const bool handled = m_editer->redo();

            if (handled && m_viewer != nullptr)
            {
                m_viewer->appendCommandMessage(QStringLiteral("重做完成"));
                m_viewer->refreshCommandPrompt();
            }

            return handled;
        }
    }

    if (event->key() == Qt::Key_Escape)
    {
        cancelDrawing();
        return true;
    }

    if (m_drawState.isDrawing && m_editer != nullptr)
    {
        if ((m_drawState.drawType == DrawType::Polyline || m_drawState.drawType == DrawType::LWPolyline)
            && event->key() == Qt::Key_A)
        {
            return setPolylineInputMode(true);
        }

        if ((m_drawState.drawType == DrawType::Polyline || m_drawState.drawType == DrawType::LWPolyline)
            && event->key() == Qt::Key_L)
        {
            return setPolylineInputMode(false);
        }

        if ((m_drawState.drawType == DrawType::Polyline || m_drawState.drawType == DrawType::LWPolyline)
            && (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter || event->key() == Qt::Key_Space))
        {
            const bool handled = m_editer->finishActivePolyline(m_drawState, false);

            if (handled && m_viewer != nullptr)
            {
                m_viewer->appendCommandMessage(QStringLiteral("已创建多段线图元"));
                m_viewer->refreshCommandPrompt();
            }

            return handled;
        }

        if ((m_drawState.drawType == DrawType::Polyline || m_drawState.drawType == DrawType::LWPolyline)
            && event->key() == Qt::Key_C)
        {
            const bool handled = m_editer->finishActivePolyline(m_drawState, true);

            if (handled && m_viewer != nullptr)
            {
                m_viewer->appendCommandMessage(QStringLiteral("已创建闭合多段线图元"));
                m_viewer->refreshCommandPrompt();
            }

            return handled;
        }
    }

    if (m_drawState.hasActiveCommand())
    {
        switch (event->key())
        {
        case Qt::Key_F:
        case Qt::Key_T:
        case Qt::Key_Home:
        case Qt::Key_Plus:
        case Qt::Key_Equal:
        case Qt::Key_Minus:
        case Qt::Key_Underscore:
        case Qt::Key_P:
        case Qt::Key_L:
        case Qt::Key_C:
        case Qt::Key_A:
        case Qt::Key_E:
        case Qt::Key_Delete:
        case Qt::Key_K:
        case Qt::Key_M:
        case Qt::Key_O:
        case Qt::Key_W:
            return true;
        default:
            break;
        }
    }

    if (event->key() == Qt::Key_Delete && m_editer != nullptr && m_viewer != nullptr)
    {
        const bool handled = m_editer->deleteEntity(m_viewer->selectedEntity());

        if (handled)
        {
            m_viewer->appendCommandMessage(QStringLiteral("已删除选中图元"));
            m_viewer->refreshCommandPrompt();
        }

        return handled;
    }

    if (event->key() == Qt::Key_M && m_editer != nullptr && m_viewer != nullptr)
    {
        const bool handled = m_editer->beginMove(m_drawState, m_viewer->selectedEntity());

        if (handled)
        {
            m_viewer->appendCommandMessage(QStringLiteral("已进入移动命令"));
            m_viewer->refreshCommandPrompt();
        }

        return handled;
    }

    if (event->key() == Qt::Key_K && m_editer != nullptr && m_viewer != nullptr)
    {
        CadItem* selectedItem = m_viewer->selectedEntity();

        if (selectedItem == nullptr)
        {
            return true;
        }

        const QColor color = QColorDialog::getColor
        (
            selectedItem->m_color,
            m_viewer,
            QStringLiteral("选择图元颜色")
        );

        if (!color.isValid())
        {
            return true;
        }

        const bool handled = m_editer->changeEntityColor(selectedItem, color);

        if (handled)
        {
            m_viewer->appendCommandMessage(QStringLiteral("已修改图元颜色"));
            m_viewer->refreshCommandPrompt();
        }

        return handled;
    }

    switch (event->key())
    {
    case Qt::Key_F:
        if (m_viewer != nullptr)
        {
            m_viewer->fitSceneView();
            return true;
        }
        break;
    case Qt::Key_T:
    case Qt::Key_Home:
        if (m_viewer != nullptr)
        {
            m_viewer->resetToTopView();
            return true;
        }
        break;
    case Qt::Key_Plus:
    case Qt::Key_Equal:
        if (m_viewer != nullptr)
        {
            m_viewer->zoomIn();
            return true;
        }
        break;
    case Qt::Key_Minus:
    case Qt::Key_Underscore:
        if (m_viewer != nullptr)
        {
            m_viewer->zoomOut();
            return true;
        }
        break;
    case Qt::Key_P:
        beginDrawing(DrawType::Point, m_drawState.drawingColor);
        return true;
    case Qt::Key_L:
        beginDrawing(DrawType::Line, m_drawState.drawingColor);
        return true;
    case Qt::Key_C:
        beginDrawing(DrawType::Circle, m_drawState.drawingColor);
        return true;
    case Qt::Key_A:
        beginDrawing(DrawType::Arc, m_drawState.drawingColor);
        return true;
    case Qt::Key_E:
        beginDrawing(DrawType::Ellipse, m_drawState.drawingColor);
        return true;
    case Qt::Key_O:
        beginDrawing(DrawType::Polyline, m_drawState.drawingColor);
        return true;
    case Qt::Key_W:
        beginDrawing(DrawType::LWPolyline, m_drawState.drawingColor);
        return true;
    default:
        break;
    }

    return false;
}

DrawStateMachine& CadController::drawState()
{
    return m_drawState;
}

const DrawStateMachine& CadController::drawState() const
{
    return m_drawState;
}

QString CadController::currentPrompt() const
{
    if (m_drawState.editType == EditType::Move)
    {
        switch (m_drawState.moveSubMode)
        {
        case MoveEditSubMode::AwaitBasePoint:
            return QStringLiteral("MOVE: 指定基点");
        case MoveEditSubMode::AwaitTargetPoint:
            return QStringLiteral("MOVE: 指定目标点");
        default:
            return QStringLiteral("MOVE");
        }
    }

    if (!m_drawState.isDrawing)
    {
        return QStringLiteral("无活动命令");
    }

    switch (m_drawState.drawType)
    {
    case DrawType::Point:
        return QStringLiteral("POINT: 指定点位置");
    case DrawType::Line:
        return m_drawState.lineSubMode == LineDrawSubMode::AwaitEndPoint
            ? QStringLiteral("LINE: 指定下一点")
            : QStringLiteral("LINE: 指定第一点");
    case DrawType::Circle:
        return m_drawState.circleSubMode == CircleDrawSubMode::AwaitRadius
            ? QStringLiteral("CIRCLE: 指定半径")
            : QStringLiteral("CIRCLE: 指定圆心");
    case DrawType::Arc:
        switch (m_drawState.arcSubMode)
        {
        case ArcDrawSubMode::AwaitRadius:
            return QStringLiteral("ARC: 指定半径");
        case ArcDrawSubMode::AwaitStartAngle:
            return QStringLiteral("ARC: 指定起始角");
        case ArcDrawSubMode::AwaitEndAngle:
            return QStringLiteral("ARC: 指定终止角");
        default:
            return QStringLiteral("ARC: 指定圆心");
        }
    case DrawType::Ellipse:
        switch (m_drawState.ellipseSubMode)
        {
        case EllipseDrawSubMode::AwaitMajorAxis:
            return QStringLiteral("ELLIPSE: 指定长轴端点");
        case EllipseDrawSubMode::AwaitMinorAxis:
            return QStringLiteral("ELLIPSE: 指定短轴距离");
        default:
            return QStringLiteral("ELLIPSE: 指定中心");
        }
    case DrawType::Polyline:
        if (m_drawState.polylineSubMode == PolylineDrawSubMode::AwaitArcEndPoint)
        {
            return QStringLiteral("POLYLINE[圆弧]: 指定圆弧终点，L切换直线，Enter结束，C闭合");
        }

        if (m_drawState.polylineSubMode == PolylineDrawSubMode::AwaitLineEndPoint)
        {
            return QStringLiteral("POLYLINE[直线]: 指定下一点，A切换圆弧，Enter结束，C闭合");
        }

        return QStringLiteral("POLYLINE: 指定第一点");
    case DrawType::LWPolyline:
        if (m_drawState.lwPolylineSubMode == LWPolylineDrawSubMode::AwaitArcEndPoint)
        {
            return QStringLiteral("LWPOLYLINE[圆弧]: 指定圆弧终点，L切换直线，Enter结束，C闭合");
        }

        if (m_drawState.lwPolylineSubMode == LWPolylineDrawSubMode::AwaitLineEndPoint)
        {
            return QStringLiteral("LWPOLYLINE[直线]: 指定下一点，A切换圆弧，Enter结束，C闭合");
        }

        return QStringLiteral("LWPOLYLINE: 指定第一点");
    default:
        break;
    }

    return QStringLiteral("无活动命令");
}

QString CadController::currentCommandName() const
{
    if (m_drawState.editType == EditType::Move)
    {
        return QStringLiteral("移动");
    }

    return drawTypeName(m_drawState.drawType);
}

void CadController::resetSubModes()
{
    m_drawState.pointSubMode = PointDrawSubMode::Idle;
    m_drawState.lineSubMode = LineDrawSubMode::Idle;
    m_drawState.circleSubMode = CircleDrawSubMode::Idle;
    m_drawState.arcSubMode = ArcDrawSubMode::Idle;
    m_drawState.ellipseSubMode = EllipseDrawSubMode::Idle;
    m_drawState.polylineSubMode = PolylineDrawSubMode::Idle;
    m_drawState.lwPolylineSubMode = LWPolylineDrawSubMode::Idle;
    m_drawState.moveSubMode = MoveEditSubMode::Idle;
}

void CadController::preparePrimitiveSubMode()
{
    switch (m_drawState.drawType)
    {
    case DrawType::Point:
        m_drawState.pointSubMode = PointDrawSubMode::AwaitPosition;
        break;
    case DrawType::Line:
        m_drawState.lineSubMode = LineDrawSubMode::AwaitStartPoint;
        break;
    case DrawType::Circle:
        m_drawState.circleSubMode = CircleDrawSubMode::AwaitCenter;
        break;
    case DrawType::Arc:
        m_drawState.arcSubMode = ArcDrawSubMode::AwaitCenter;
        break;
    case DrawType::Ellipse:
        m_drawState.ellipseSubMode = EllipseDrawSubMode::AwaitCenter;
        break;
    case DrawType::Polyline:
        m_drawState.polylineSubMode = PolylineDrawSubMode::AwaitFirstPoint;
        break;
    case DrawType::LWPolyline:
        m_drawState.lwPolylineSubMode = LWPolylineDrawSubMode::AwaitFirstPoint;
        break;
    default:
        break;
    }
}

void CadController::handleLeftPressInCommand(const QVector3D& worldPos)
{
    if (!m_drawState.hasActiveCommand())
    {
        return;
    }

    m_drawState.lastPos = worldPos;
    m_drawState.currentPos = worldPos;

    if (m_drawState.editType == EditType::Move)
    {
        if (m_drawState.moveSubMode == MoveEditSubMode::AwaitBasePoint)
        {
            m_drawState.moveSubMode = MoveEditSubMode::AwaitTargetPoint;
        }
        else
        {
            m_drawState.moveSubMode = MoveEditSubMode::Idle;
        }

        return;
    }

    switch (m_drawState.drawType)
    {
    case DrawType::Point:
        m_drawState.pointSubMode = PointDrawSubMode::AwaitPosition;
        break;
    case DrawType::Line:
        if (m_drawState.lineSubMode == LineDrawSubMode::AwaitStartPoint)
        {
            m_drawState.lineSubMode = LineDrawSubMode::AwaitEndPoint;
        }
        else
        {
            m_drawState.lineSubMode = LineDrawSubMode::AwaitStartPoint;
        }
        break;
    case DrawType::Circle:
        if (m_drawState.circleSubMode == CircleDrawSubMode::AwaitCenter)
        {
            m_drawState.circleSubMode = CircleDrawSubMode::AwaitRadius;
        }
        else
        {
            m_drawState.circleSubMode = CircleDrawSubMode::AwaitCenter;
        }
        break;
    case DrawType::Arc:
        switch (m_drawState.arcSubMode)
        {
        case ArcDrawSubMode::AwaitCenter:
            m_drawState.arcSubMode = ArcDrawSubMode::AwaitRadius;
            break;
        case ArcDrawSubMode::AwaitRadius:
            m_drawState.arcSubMode = ArcDrawSubMode::AwaitStartAngle;
            break;
        case ArcDrawSubMode::AwaitStartAngle:
            m_drawState.arcSubMode = ArcDrawSubMode::AwaitEndAngle;
            break;
        default:
            m_drawState.arcSubMode = ArcDrawSubMode::AwaitCenter;
            break;
        }
        break;
    case DrawType::Ellipse:
        switch (m_drawState.ellipseSubMode)
        {
        case EllipseDrawSubMode::AwaitCenter:
            m_drawState.ellipseSubMode = EllipseDrawSubMode::AwaitMajorAxis;
            break;
        case EllipseDrawSubMode::AwaitMajorAxis:
            m_drawState.ellipseSubMode = EllipseDrawSubMode::AwaitMinorAxis;
            break;
        default:
            m_drawState.ellipseSubMode = EllipseDrawSubMode::AwaitCenter;
            break;
        }
        break;
    case DrawType::Polyline:
        if (m_drawState.polylineSubMode == PolylineDrawSubMode::AwaitFirstPoint)
        {
            m_drawState.polylineSubMode = m_drawState.polylineArcMode
                ? PolylineDrawSubMode::AwaitArcEndPoint
                : PolylineDrawSubMode::AwaitLineEndPoint;
        }
        break;
    case DrawType::LWPolyline:
        if (m_drawState.lwPolylineSubMode == LWPolylineDrawSubMode::AwaitFirstPoint)
        {
            m_drawState.lwPolylineSubMode = m_drawState.lwPolylineArcMode
                ? LWPolylineDrawSubMode::AwaitArcEndPoint
                : LWPolylineDrawSubMode::AwaitLineEndPoint;
        }
        break;
    default:
        break;
    }
}

bool CadController::isPolylineCommandActive() const
{
    return m_drawState.isDrawing
        && (m_drawState.drawType == DrawType::Polyline || m_drawState.drawType == DrawType::LWPolyline);
}

bool CadController::setPolylineInputMode(bool useArc)
{
    if (!isPolylineCommandActive())
    {
        return false;
    }

    const bool lightweight = m_drawState.drawType == DrawType::LWPolyline;
    QVector<QVector3D>& commandPoints = m_drawState.commandPoints;

    if (useArc && commandPoints.size() < 2)
    {
        if (m_viewer != nullptr)
        {
            m_viewer->appendCommandMessage(QStringLiteral("圆弧段需要先确定至少一段前置线段"));
            m_viewer->refreshCommandPrompt();
        }

        return true;
    }

    if (lightweight)
    {
        m_drawState.lwPolylineArcMode = useArc;

        if (m_drawState.lwPolylineSubMode != LWPolylineDrawSubMode::AwaitFirstPoint)
        {
            m_drawState.lwPolylineSubMode = useArc ? LWPolylineDrawSubMode::AwaitArcEndPoint : LWPolylineDrawSubMode::AwaitLineEndPoint;
        }
    }
    else
    {
        m_drawState.polylineArcMode = useArc;

        if (m_drawState.polylineSubMode != PolylineDrawSubMode::AwaitFirstPoint)
        {
            m_drawState.polylineSubMode = useArc ? PolylineDrawSubMode::AwaitArcEndPoint : PolylineDrawSubMode::AwaitLineEndPoint;
        }
    }

    if (m_viewer != nullptr)
    {
        m_viewer->appendCommandMessage(useArc ? QStringLiteral("已切换到圆弧段输入") : QStringLiteral("已切换到直线段输入"));
        m_viewer->refreshCommandPrompt();
    }

    return true;
}

QVector3D CadController::currentWorldPos(const QPoint& screenPos) const
{
    if (m_viewer == nullptr)
    {
        return QVector3D();
    }

    if (m_drawState.hasActiveCommand())
    {
        return m_viewer->screenToGroundPlane(screenPos);
    }

    return m_viewer->screenToWorld(screenPos);
}
