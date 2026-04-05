#include "pch.h"

#include "CadController.h"

#include "CadViewer.h"

void CadController::setViewer(CadViewer* viewer)
{
    m_viewer = viewer;
}

void CadController::reset()
{
    m_drawState.reset();
}

void CadController::beginDrawing(DrawType drawType, const QColor& color)
{
    m_drawState.isDrawing = true;
    m_drawState.drawType = drawType;
    m_drawState.drawingColor = color;
    resetSubModes();
    preparePrimitiveSubMode();
}

void CadController::cancelDrawing()
{
    m_drawState.isDrawing = false;
    m_drawState.drawType = DrawType::None;
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

    if (event->button() == Qt::LeftButton)
    {
        handleLeftPressInDrawing(worldPos);
    }

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

    // 绘图过程中屏蔽视图快捷键，避免与图元命令冲突。
    if (m_drawState.isDrawing)
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
            return true;
        default:
            break;
        }
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
    case Qt::Key_Escape:
        cancelDrawing();
        return true;
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

void CadController::handleLeftPressInDrawing(const QVector3D& worldPos)
{
    if (!m_drawState.isDrawing)
    {
        return;
    }

    m_drawState.lastPos = worldPos;
    m_drawState.currentPos = worldPos;

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

    return m_viewer->screenToWorld(screenPos);
}
