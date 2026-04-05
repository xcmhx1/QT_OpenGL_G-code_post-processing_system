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
    resetSubModes();
    preparePrimitiveSubMode();
}

void CadController::cancelDrawing()
{
    if (m_editer != nullptr)
    {
        m_editer->cancelTransientCommand();
    }

    m_drawState.isDrawing = false;
    m_drawState.drawType = DrawType::None;
    m_drawState.editType = EditType::None;
    m_drawState.commandPoints.clear();
    resetSubModes();
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
            return true;
        }

        m_viewer->selectEntityAt(event->pos());
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
            return m_editer->redo();
        }

        if (event->key() == Qt::Key_Z)
        {
            return m_editer->undo();
        }

        if (event->key() == Qt::Key_Y)
        {
            return m_editer->redo();
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
            && (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter || event->key() == Qt::Key_Space))
        {
            return m_editer->finishActivePolyline(m_drawState, false);
        }

        if ((m_drawState.drawType == DrawType::Polyline || m_drawState.drawType == DrawType::LWPolyline)
            && event->key() == Qt::Key_C)
        {
            return m_editer->finishActivePolyline(m_drawState, true);
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
        return m_editer->deleteEntity(m_viewer->selectedEntity());
    }

    if (event->key() == Qt::Key_M && m_editer != nullptr && m_viewer != nullptr)
    {
        return m_editer->beginMove(m_drawState, m_viewer->selectedEntity());
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

        return m_editer->changeEntityColor(selectedItem, color);
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
            m_drawState.polylineSubMode = PolylineDrawSubMode::AwaitLineEndPoint;
        }
        break;
    case DrawType::LWPolyline:
        if (m_drawState.lwPolylineSubMode == LWPolylineDrawSubMode::AwaitFirstPoint)
        {
            m_drawState.lwPolylineSubMode = LWPolylineDrawSubMode::AwaitLineEndPoint;
        }
        break;
    default:
        break;
    }
}

QVector3D CadController::currentWorldPos(const QPoint& screenPos) const
{
    if (m_viewer == nullptr)
    {
        return QVector3D();
    }

    if (m_drawState.isDrawing)
    {
        const QVector3D nearPoint = m_viewer->screenToWorld(screenPos, -1.0f);
        const QVector3D farPoint = m_viewer->screenToWorld(screenPos, 1.0f);
        const QVector3D rayDirection = farPoint - nearPoint;

        if (!qFuzzyIsNull(rayDirection.z()))
        {
            const float t = -nearPoint.z() / rayDirection.z();
            return nearPoint + rayDirection * t;
        }

        return flattenToDrawingPlane(nearPoint);
    }

    return m_viewer->screenToWorld(screenPos);
}
