// CadController 实现文件
// 实现 CadController 模块，对应头文件中声明的主要行为和协作流程。
// 输入控制模块，负责解释键盘、鼠标、滚轮事件并驱动绘图/编辑命令。

#include "pch.h"

#include "CadController.h"

// Qt 核心模块
#include <QColorDialog>

// CAD 模块内部依赖
#include "CadEditer.h"
#include "CadItem.h"
#include "CadProcessVisualUtils.h"
#include "CadViewer.h"

// 匿名命名空间，存放局部辅助函数
namespace
{
    // 将三维点投影到绘图平面（XY平面）
    // @param point 三维点
    // @return 投影到XY平面的点（Z坐标为0）
    QVector3D flattenToDrawingPlane(const QVector3D& point)
    {
        return QVector3D(point.x(), point.y(), 0.0f);
    }

    // 将绘制类型转换为对应的中文名称
    // @param drawType 绘制类型
    // @return 对应的中文名称字符串
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

// 设置关联的视图对象
// @param viewer CAD 视图对象指针
void CadController::setViewer(CadViewer* viewer)
{
    m_viewer = viewer;
}

// 设置关联的编辑器对象
// @param editer CAD 编辑器对象指针
void CadController::setEditer(CadEditer* editer)
{
    m_editer = editer;
}

// 重置控制器状态
void CadController::reset()
{
    // 如果有编辑器，取消其临时命令
    if (m_editer != nullptr)
    {
        m_editer->cancelTransientCommand();
    }

    // 重置绘图状态机
    m_drawState.reset();

    // 如果有视图，刷新命令提示
    if (m_viewer != nullptr)
    {
        m_viewer->refreshCommandPrompt();
    }
}

void CadController::setDefaultDrawingProperties(const QString& layerName, const QColor& color, int colorIndex)
{
    m_drawState.drawingLayerName = layerName.trimmed().isEmpty() ? QStringLiteral("0") : layerName.trimmed();
    m_drawState.drawingColor = color.isValid() ? color : QColor(Qt::white);
    m_drawState.drawingColorIndex = colorIndex;
}

// 开始绘制指定类型的图元
// @param drawType 绘制类型
// @param color 图元颜色，默认为白色
void CadController::beginDrawing(DrawType drawType, const QColor& color)
{
    // 如果有编辑器，取消其临时命令
    if (m_editer != nullptr)
    {
        m_editer->cancelTransientCommand();
    }

    // 设置绘图状态
    m_drawState.isDrawing = true;
    m_drawState.drawType = drawType;
    m_drawState.drawingColor = color;
    m_drawState.editType = EditType::None;
    m_drawState.commandPoints.clear();
    m_drawState.commandBulges.clear();
    m_drawState.polylineArcMode = false;
    m_drawState.lwPolylineArcMode = false;

    // 重置子模式
    resetSubModes();
    // 准备图元子模式
    preparePrimitiveSubMode();

    // 如果有视图，发送消息并刷新提示
    if (m_viewer != nullptr)
    {
        m_viewer->appendCommandMessage(QStringLiteral("已进入%1命令").arg(drawTypeName(drawType)));
        m_viewer->refreshCommandPrompt();
    }
}

bool CadController::beginMoveSelected()
{
    if (m_editer == nullptr || m_viewer == nullptr)
    {
        return false;
    }

    const bool handled = m_editer->beginMove(m_drawState, m_viewer->selectedEntity());

    if (handled)
    {
        m_viewer->appendCommandMessage(QStringLiteral("已进入移动命令"));
        m_viewer->refreshCommandPrompt();
    }

    return handled;
}

// 取消当前绘制操作
void CadController::cancelDrawing()
{
    // 记录是否有活动命令
    const bool hadActiveCommand = m_drawState.hasActiveCommand();

    // 如果有编辑器，取消其临时命令
    if (m_editer != nullptr)
    {
        m_editer->cancelTransientCommand();
    }

    // 重置绘图状态
    m_drawState.isDrawing = false;
    m_drawState.drawType = DrawType::None;
    m_drawState.editType = EditType::None;
    m_drawState.commandPoints.clear();
    m_drawState.commandBulges.clear();
    m_drawState.polylineArcMode = false;
    m_drawState.lwPolylineArcMode = false;

    // 重置子模式
    resetSubModes();

    // 如果有视图，发送取消消息并刷新提示
    if (m_viewer != nullptr)
    {
        if (hadActiveCommand)
        {
            m_viewer->appendCommandMessage(QStringLiteral("命令已取消"));
        }

        m_viewer->refreshCommandPrompt();
    }
}

// 处理鼠标按下事件
// @param event 鼠标事件
// @return 如果事件被处理返回 true，否则返回 false
bool CadController::handleMousePress(QMouseEvent* event)
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    // 获取当前世界坐标
    const QVector3D worldPos = currentWorldPos(event->pos());

    // 更新绘图状态机中的鼠标信息
    m_drawState.pressScreenPos = event->pos();
    m_drawState.lastScreenPos = event->pos();
    m_drawState.currentScreenPos = event->pos();
    m_drawState.activeButton = event->button();
    m_drawState.pressedButtons = event->buttons();
    m_drawState.keyboardModifiers = event->modifiers();
    m_drawState.lastPos = m_drawState.currentPos;
    m_drawState.currentPos = worldPos;

    // 处理中键按下：视图操作
    if (event->button() == Qt::MiddleButton)
    {
        if ((event->modifiers() & Qt::ShiftModifier) != 0)
        {
            // 中键+Shift：开始轨道旋转
            m_viewer->beginOrbitInteraction();
        }
        else
        {
            // 中键：开始平移
            m_viewer->beginPanInteraction();
        }

        return true;
    }

    // 处理左键按下：选择或绘图
    if (event->button() == Qt::LeftButton)
    {
        // 保存之前的状态用于比较
        const DrawStateMachine previousState = m_drawState;

        // 处理命令状态下的左键按下
        handleLeftPressInCommand(worldPos);

        // 尝试由编辑器处理左键按下事件
        if (m_editer != nullptr && m_editer->handleLeftPress(previousState, m_drawState, worldPos))
        {
            // 如果编辑器处理了事件，根据状态变化发送相应消息
            if (m_viewer != nullptr)
            {
                if (previousState.editType == EditType::Move && m_drawState.editType == EditType::None)
                {
                    m_viewer->appendCommandMessage(QStringLiteral("移动完成"));
                }
                else if (previousState.editType == EditType::GripEdit && m_drawState.editType == EditType::None)
                {
                    m_viewer->appendCommandMessage(QStringLiteral("控制点编辑完成"));
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

        // 空闲状态下优先尝试命中当前选中图元的可编辑控制点
        if (!m_drawState.hasActiveCommand() && m_editer != nullptr)
        {
            CadSelectionHandleInfo handleInfo;

            if (m_viewer->pickSelectedHandle(event->pos(), &handleInfo))
            {
                if (m_editer->beginGripEdit(m_drawState, m_viewer->selectedEntity(), handleInfo))
                {
                    m_viewer->appendCommandMessage(QStringLiteral("已进入控制点编辑"));
                    m_viewer->refreshCommandPrompt();
                    return true;
                }
            }
        }

        // 如果编辑器未处理，执行实体选择
        m_viewer->selectEntityAt(event->pos());
        m_viewer->refreshCommandPrompt();
        return true;
    }

    return false;
}

// 处理鼠标移动事件
// @param event 鼠标事件
// @return 如果事件被处理返回 true，否则返回 false
bool CadController::handleMouseMove(QMouseEvent* event)
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    // 获取当前世界坐标
    const QVector3D worldPos = currentWorldPos(event->pos());

    // 更新绘图状态机中的鼠标信息
    m_drawState.lastScreenPos = m_drawState.currentScreenPos;
    m_drawState.currentScreenPos = event->pos();
    m_drawState.pressedButtons = event->buttons();
    m_drawState.keyboardModifiers = event->modifiers();
    m_drawState.lastPos = m_drawState.currentPos;
    m_drawState.currentPos = worldPos;

    // 如果是轨道交互且需要忽略下一次增量，则消费此标志
    if (m_viewer->interactionMode() == ViewInteractionMode::Orbiting && m_viewer->shouldIgnoreNextOrbitDelta())
    {
        m_viewer->consumeIgnoreNextOrbitDelta();
        return true;
    }

    // 计算鼠标移动增量
    const QPoint delta = m_drawState.currentScreenPos - m_drawState.lastScreenPos;

    // 根据当前交互模式处理移动
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

// 处理鼠标释放事件
// @param event 鼠标事件
// @return 如果事件被处理返回 true，否则返回 false
bool CadController::handleMouseRelease(QMouseEvent* event)
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    // 更新绘图状态机中的鼠标信息
    m_drawState.lastScreenPos = m_drawState.currentScreenPos;
    m_drawState.currentScreenPos = event->pos();
    m_drawState.activeButton = event->button();
    m_drawState.pressedButtons = event->buttons();
    m_drawState.keyboardModifiers = event->modifiers();

    // 处理中键释放：结束视图交互
    if (event->button() == Qt::MiddleButton)
    {
        m_viewer->endViewInteraction();
        return true;
    }

    return false;
}

// 处理滚轮事件
// @param event 滚轮事件
// @return 如果事件被处理返回 true，否则返回 false
bool CadController::handleWheel(QWheelEvent* event)
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    // 获取当前世界坐标
    const QVector3D worldPos = currentWorldPos(event->position().toPoint());
    m_drawState.currentPos = worldPos;
    m_drawState.keyboardModifiers = event->modifiers();

    // 计算缩放因子
    const float factor = event->angleDelta().y() > 0 ? 1.1f : (1.0f / 1.1f);

    // 在鼠标位置缩放
    m_viewer->zoomAtScreenPosition(event->position().toPoint(), factor);
    event->accept();
    return true;
}

// 处理键盘按下事件
// @param event 键盘事件
// @return 如果事件被处理返回 true，否则返回 false
bool CadController::handleKeyPress(QKeyEvent* event)
{
    // 更新键盘修饰符
    m_drawState.keyboardModifiers = event->modifiers();

    // 处理Ctrl组合键：撤销/重做
    if ((event->modifiers() & Qt::ControlModifier) != 0 && m_editer != nullptr)
    {
        // Ctrl+Shift+Z 或 Ctrl+Y：重做
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

        // Ctrl+Z：撤销
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

        // Ctrl+Y：重做
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

    // ESC键：优先取消当前命令；无活动命令时清空当前选中
    if (event->key() == Qt::Key_Escape)
    {
        if (m_drawState.hasActiveCommand())
        {
            cancelDrawing();
            return true;
        }

        if (m_viewer != nullptr && m_viewer->selectedEntity() != nullptr)
        {
            m_viewer->clearSelection();
            m_viewer->appendCommandMessage(QStringLiteral("已取消选中"));
            m_viewer->refreshCommandPrompt();
        }

        return true;
    }

    // 在绘图状态下处理多段线相关按键
    if (m_drawState.isDrawing && m_editer != nullptr)
    {
        // 多段线：A键切换到圆弧模式
        if ((m_drawState.drawType == DrawType::Polyline || m_drawState.drawType == DrawType::LWPolyline)
            && event->key() == Qt::Key_A)
        {
            return setPolylineInputMode(true);
        }

        // 多段线：L键切换到直线模式
        if ((m_drawState.drawType == DrawType::Polyline || m_drawState.drawType == DrawType::LWPolyline)
            && event->key() == Qt::Key_L)
        {
            return setPolylineInputMode(false);
        }

        // 多段线：Enter或空格键完成多段线（不闭合）
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

        // 多段线：C键闭合多段线
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

    // 在有活动命令时处理其他快捷键
    if (m_drawState.hasActiveCommand())
    {
        switch (event->key())
        {
        case Qt::Key_F:     // 适配视图
        case Qt::Key_T:     // 顶视图
        case Qt::Key_Home:  // 顶视图
        case Qt::Key_Plus:  // 放大
        case Qt::Key_Equal: // 放大
        case Qt::Key_Minus: // 缩小
        case Qt::Key_Underscore: // 缩小
        case Qt::Key_P:     // 点
        case Qt::Key_L:     // 直线
        case Qt::Key_C:     // 圆
        case Qt::Key_A:     // 圆弧
        case Qt::Key_E:     // 椭圆
        case Qt::Key_Delete:// 删除
        case Qt::Key_K:     // 颜色
        case Qt::Key_M:     // 移动
        case Qt::Key_O:     // 多段线
        case Qt::Key_W:     // 轻量多段线
            return true;
        default:
            break;
        }
    }

    // Delete键：删除选中实体
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

    // M键：开始移动命令
    if (event->key() == Qt::Key_M && m_editer != nullptr && m_viewer != nullptr)
    {
        return beginMoveSelected();
    }

    // K键：修改选中实体颜色
    if (event->key() == Qt::Key_K && m_editer != nullptr && m_viewer != nullptr)
    {
        CadItem* selectedItem = m_viewer->selectedEntity();

        if (selectedItem == nullptr)
        {
            return true;
        }

        // 打开颜色选择对话框
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

    // 处理其他功能键
    switch (event->key())
    {
    case Qt::Key_F:  // 适配视图
        if (m_viewer != nullptr)
        {
            m_viewer->fitSceneView();
            return true;
        }
        break;
    case Qt::Key_T:  // 顶视图
    case Qt::Key_Home:  // 顶视图
        if (m_viewer != nullptr)
        {
            m_viewer->resetToTopView();
            return true;
        }
        break;
    case Qt::Key_Plus:  // 放大
    case Qt::Key_Equal:  // 放大
        if (m_viewer != nullptr)
        {
            m_viewer->zoomIn();
            return true;
        }
        break;
    case Qt::Key_Minus:  // 缩小
    case Qt::Key_Underscore:  // 缩小
        if (m_viewer != nullptr)
        {
            m_viewer->zoomOut();
            return true;
        }
        break;
    case Qt::Key_P:  // 开始绘制点
        beginDrawing(DrawType::Point, m_drawState.drawingColor);
        return true;
    case Qt::Key_L:  // 开始绘制直线
        beginDrawing(DrawType::Line, m_drawState.drawingColor);
        return true;
    case Qt::Key_C:  // 开始绘制圆
        beginDrawing(DrawType::Circle, m_drawState.drawingColor);
        return true;
    case Qt::Key_A:  // 开始绘制圆弧
        beginDrawing(DrawType::Arc, m_drawState.drawingColor);
        return true;
    case Qt::Key_E:  // 开始绘制椭圆
        beginDrawing(DrawType::Ellipse, m_drawState.drawingColor);
        return true;
    case Qt::Key_O:  // 开始绘制多段线
        beginDrawing(DrawType::Polyline, m_drawState.drawingColor);
        return true;
    case Qt::Key_W:  // 开始绘制轻量多段线
        beginDrawing(DrawType::LWPolyline, m_drawState.drawingColor);
        return true;
    default:
        break;
    }

    return false;
}

// 获取绘图状态机（可修改）
// @return 绘图状态机引用
DrawStateMachine& CadController::drawState()
{
    return m_drawState;
}

// 获取绘图状态机（只读）
// @return 绘图状态机常量引用
const DrawStateMachine& CadController::drawState() const
{
    return m_drawState;
}

// 获取当前命令提示文本
// @return 命令提示字符串
QString CadController::currentPrompt() const
{
    // 编辑命令提示
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

    if (m_drawState.editType == EditType::GripEdit)
    {
        return QStringLiteral("GRIP: 指定目标点");
    }

    // 无活动命令提示
    if (!m_drawState.isDrawing)
    {
        return QStringLiteral("无活动命令");
    }

    // 根据当前绘制类型和子模式返回相应提示
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

// 获取当前命令名称
// @return 命令名称字符串
QString CadController::currentCommandName() const
{
    if (m_drawState.editType == EditType::Move)
    {
        return QStringLiteral("移动");
    }

    if (m_drawState.editType == EditType::GripEdit)
    {
        return QStringLiteral("控制点编辑");
    }

    return drawTypeName(m_drawState.drawType);
}

// 重置所有子模式
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
    m_drawState.gripSubMode = GripEditSubMode::Idle;
}

// 准备图元子模式（进入绘图状态）
void CadController::preparePrimitiveSubMode()
{
    // 根据绘制类型设置相应的初始子模式
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

// 处理命令状态下的鼠标左键按下
// @param worldPos 世界坐标位置
void CadController::handleLeftPressInCommand(const QVector3D& worldPos)
{
    // 如果没有活动命令，则返回
    if (!m_drawState.hasActiveCommand())
    {
        return;
    }

    // 更新位置信息
    m_drawState.lastPos = worldPos;
    m_drawState.currentPos = worldPos;

    // 处理移动编辑
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

    if (m_drawState.editType == EditType::GripEdit)
    {
        if (m_drawState.gripSubMode == GripEditSubMode::AwaitTargetPoint)
        {
            m_drawState.gripSubMode = GripEditSubMode::Idle;
        }

        return;
    }

    // 根据绘制类型处理左键按下
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

// 检查是否有多段线命令处于活动状态
// @return 如果多段线命令活动则返回 true
bool CadController::isPolylineCommandActive() const
{
    return m_drawState.isDrawing
        && (m_drawState.drawType == DrawType::Polyline || m_drawState.drawType == DrawType::LWPolyline);
}

// 设置多段线输入模式
// @param useArc 是否使用圆弧模式
// @return 如果成功设置模式返回 true
bool CadController::setPolylineInputMode(bool useArc)
{
    if (!isPolylineCommandActive())
    {
        return false;
    }

    const bool lightweight = m_drawState.drawType == DrawType::LWPolyline;
    QVector<QVector3D>& commandPoints = m_drawState.commandPoints;

    // 圆弧模式需要至少两个点（一段前置线段）
    if (useArc && commandPoints.size() < 2)
    {
        if (m_viewer != nullptr)
        {
            m_viewer->appendCommandMessage(QStringLiteral("圆弧段需要先确定至少一段前置线段"));
            m_viewer->refreshCommandPrompt();
        }

        return true;
    }

    // 根据多段线类型设置相应模式
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

    // 发送模式切换消息
    if (m_viewer != nullptr)
    {
        m_viewer->appendCommandMessage(useArc ? QStringLiteral("已切换到圆弧段输入") : QStringLiteral("已切换到直线段输入"));
        m_viewer->refreshCommandPrompt();
    }

    return true;
}

// 将屏幕坐标转换为当前世界坐标
// @param screenPos 屏幕坐标
// @return 对应的世界坐标
QVector3D CadController::currentWorldPos(const QPoint& screenPos) const
{
    if (m_viewer == nullptr)
    {
        return QVector3D();
    }

    return m_viewer->resolveInteractiveWorldPosition(screenPos);
}
