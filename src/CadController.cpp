// CadController 实现文件
// 实现 CadController 模块，对应头文件中声明的主要行为和协作流程。
// 输入控制模块，负责解释键盘、鼠标、滚轮事件并驱动绘图/编辑命令。

#include "pch.h"

#include "CadController.h"

// Qt 核心模块
#include <QColorDialog>
#include <QCursor>

// CAD 模块内部依赖
#include "CadEditer.h"
#include "CadItem.h"
#include "CadProcessVisualUtils.h"
#include "CadViewer.h"

// 标准库
#include <algorithm>
#include <cmath>

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
        case DrawType::Xline:
            return QStringLiteral("构造线");
        case DrawType::Rectangle:
            return QStringLiteral("矩形");
        case DrawType::Polygon:
            return QStringLiteral("多边形");
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

    QString polygonConstructionModeText(bool circumscribedAboutCircle)
    {
        return circumscribedAboutCircle
            ? QStringLiteral("外切于圆")
            : QStringLiteral("内切于圆");
    }

    QVector3D geometryBoundsCenter(const QVector<CadItem*>& items)
    {
        if (items.isEmpty())
        {
            return QVector3D();
        }

        QVector3D minPoint;
        QVector3D maxPoint;
        bool initialized = false;

        for (const CadItem* item : items)
        {
            if (item == nullptr || item->m_geometry.vertices.isEmpty())
            {
                continue;
            }

            for (const QVector3D& point : item->m_geometry.vertices)
            {
                if (!initialized)
                {
                    minPoint = point;
                    maxPoint = point;
                    initialized = true;
                    continue;
                }

                minPoint.setX(std::min(minPoint.x(), point.x()));
                minPoint.setY(std::min(minPoint.y(), point.y()));
                minPoint.setZ(std::min(minPoint.z(), point.z()));
                maxPoint.setX(std::max(maxPoint.x(), point.x()));
                maxPoint.setY(std::max(maxPoint.y(), point.y()));
                maxPoint.setZ(std::max(maxPoint.z(), point.z()));
            }
        }

        if (!initialized)
        {
            return QVector3D();
        }

        return QVector3D
        (
            (minPoint.x() + maxPoint.x()) * 0.5f,
            (minPoint.y() + maxPoint.y()) * 0.5f,
            (minPoint.z() + maxPoint.z()) * 0.5f
        );
    }

    bool resolveCurrentAndOtherSelectedItems(CadViewer* viewer, CadItem*& currentItem, CadItem*& otherItem)
    {
        currentItem = nullptr;
        otherItem = nullptr;

        if (viewer == nullptr)
        {
            return false;
        }

        const QVector<CadItem*> selectedItems = viewer->selectedEntities();

        if (selectedItems.size() != 2)
        {
            return false;
        }

        currentItem = viewer->selectedEntity();

        if (currentItem == nullptr || !selectedItems.contains(currentItem))
        {
            currentItem = selectedItems.front();
        }

        otherItem = selectedItems.front() == currentItem ? selectedItems.back() : selectedItems.front();
        return currentItem != nullptr && otherItem != nullptr;
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
    clearDynamicCommandMode();
    resetParameterInputSession();
    m_idleWindowSelectionTracking = false;
    m_idleWindowSelectionDragging = false;
    m_idleWindowSelectionAnchor = QPoint();
    m_idleWindowSelectionCurrent = QPoint();

    // 如果有视图，刷新命令提示
    if (m_viewer != nullptr)
    {
        m_viewer->hideSelectionWindowPreview();
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
    if (drawType == DrawType::Polygon)
    {
        beginPolygonConfiguration(color);
        return;
    }

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
    clearDynamicCommandMode();
    resetParameterInputSession();
    resetPointDynamicInputSession();

    // 重置子模式
    resetSubModes();
    // 准备图元子模式
    preparePrimitiveSubMode();
    resetPointDynamicInputSession(currentPointInputStageKey());

    // 如果有视图，发送消息并刷新提示
    if (m_viewer != nullptr)
    {
        if (drawType == DrawType::Polygon)
        {
            m_viewer->appendCommandMessage
            (
                QStringLiteral("已进入多边形命令（%1 边，%2）")
                .arg(m_drawState.polygonSideCount)
                .arg(polygonConstructionModeText(m_drawState.polygonCircumscribedAboutCircle))
            );
        }
        else
        {
            m_viewer->appendCommandMessage(QStringLiteral("已进入%1命令").arg(drawTypeName(drawType)));
        }

        m_viewer->refreshCommandPrompt();
    }
}

bool CadController::configurePolygonDrawing()
{
    return beginPolygonConfiguration(m_drawState.drawingColor);
}

bool CadController::beginMoveSelected()
{
    if (m_editer == nullptr || m_viewer == nullptr)
    {
        return false;
    }

    const QVector<CadItem*> selectedItems = m_viewer->selectedEntities();
    const bool handled = m_editer->beginMove(m_drawState, selectedItems);

    if (handled)
    {
        clearDynamicCommandMode();
        resetParameterInputSession();
        resetPointDynamicInputSession(currentPointInputStageKey());
        m_viewer->appendCommandMessage
        (
            selectedItems.size() > 1
                ? QStringLiteral("已进入移动命令（%1 个图元）").arg(selectedItems.size())
                : QStringLiteral("已进入移动命令")
        );
        m_viewer->refreshCommandPrompt();
    }

    return handled;
}

bool CadController::beginPolygonConfiguration(const QColor& color)
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    cancelDrawing();
    m_drawState.isDrawing = false;
    m_drawState.drawType = DrawType::None;
    m_drawState.editType = EditType::ParameterInput;
    m_drawState.currentPos = currentWorldPos(m_drawState.currentScreenPos);
    clearDynamicCommandMode();
    resetPointDynamicInputSession();
    resetParameterInputSession();
    m_parameterInputSession.command = ParameterInputCommand::Polygon;
    m_parameterInputSession.drawingColor = color;
    m_parameterInputSession.intValue1 = std::clamp(m_drawState.polygonSideCount, 3, 1024);
    m_parameterInputSession.boolValue = m_drawState.polygonCircumscribedAboutCircle;
    activateParameterInputSession();
    m_viewer->appendCommandMessage(QStringLiteral("多边形: 请在光标旁输入边数和构造方式"));
    return true;
}

void CadController::activateParameterInputSession()
{
    m_drawState.isDrawing = false;
    m_drawState.drawType = DrawType::None;
    m_drawState.editType = EditType::ParameterInput;
    m_drawState.commandPoints.clear();
    m_drawState.commandBulges.clear();
    m_drawState.polylineArcMode = false;
    m_drawState.lwPolylineArcMode = false;
    m_parameterInputSession.stageIndex = std::max(0, m_parameterInputSession.stageIndex);
    m_parameterInputSession.fieldBuffer.clear();
    resetPointDynamicInputSession(currentPointInputStageKey());

    if (m_viewer != nullptr)
    {
        syncCurrentPosWithCursor();
        m_viewer->refreshCommandPrompt();
        m_viewer->requestViewUpdate();
    }
}

void CadController::resetParameterInputSession()
{
    m_parameterInputSession = ParameterInputSession();
}

bool CadController::beginCopySelected()
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    const QVector<CadItem*> selectedItems = m_viewer->selectedEntities();

    if (selectedItems.isEmpty())
    {
        return false;
    }

    cancelDrawing();
    m_parameterInputSession.command = ParameterInputCommand::Copy;
    m_parameterInputSession.selectedItems = selectedItems;
    m_parameterInputSession.doubleValue1 = 10.0;
    m_parameterInputSession.doubleValue2 = 10.0;
    activateParameterInputSession();
    m_viewer->appendCommandMessage(QStringLiteral("复制: 请在光标旁输入 X/Y 偏移量"));
    return true;
}

bool CadController::beginRotateSelected()
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    const QVector<CadItem*> selectedItems = m_viewer->selectedEntities();

    if (selectedItems.isEmpty())
    {
        return false;
    }

    cancelDrawing();
    m_parameterInputSession.command = ParameterInputCommand::Rotate;
    m_parameterInputSession.selectedItems = selectedItems;
    m_parameterInputSession.centerPoint = geometryBoundsCenter(selectedItems);
    m_parameterInputSession.doubleValue1 = 90.0;
    activateParameterInputSession();
    m_viewer->appendCommandMessage(QStringLiteral("旋转: 请在光标旁输入角度"));
    return true;
}

bool CadController::beginScaleSelected()
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    const QVector<CadItem*> selectedItems = m_viewer->selectedEntities();

    if (selectedItems.isEmpty())
    {
        return false;
    }

    cancelDrawing();
    m_parameterInputSession.command = ParameterInputCommand::Scale;
    m_parameterInputSession.selectedItems = selectedItems;
    m_parameterInputSession.centerPoint = geometryBoundsCenter(selectedItems);
    m_parameterInputSession.doubleValue1 = 2.0;
    activateParameterInputSession();
    m_viewer->appendCommandMessage(QStringLiteral("缩放: 请在光标旁输入倍率"));
    return true;
}

bool CadController::beginRectangularArraySelected()
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    const QVector<CadItem*> selectedItems = m_viewer->selectedEntities();

    if (selectedItems.isEmpty())
    {
        return false;
    }

    cancelDrawing();
    m_parameterInputSession.command = ParameterInputCommand::RectangularArray;
    m_parameterInputSession.selectedItems = selectedItems;
    m_parameterInputSession.intValue1 = 2;
    m_parameterInputSession.intValue2 = 2;
    m_parameterInputSession.doubleValue1 = 10.0;
    m_parameterInputSession.doubleValue2 = 10.0;
    activateParameterInputSession();
    m_viewer->appendCommandMessage(QStringLiteral("矩形阵列: 请依次输入行数、列数、行间距、列间距"));
    return true;
}

bool CadController::beginCircularArraySelected()
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    const QVector<CadItem*> selectedItems = m_viewer->selectedEntities();

    if (selectedItems.isEmpty())
    {
        return false;
    }

    cancelDrawing();
    m_parameterInputSession.command = ParameterInputCommand::CircularArray;
    m_parameterInputSession.selectedItems = selectedItems;
    m_parameterInputSession.centerPoint = geometryBoundsCenter(selectedItems);
    m_parameterInputSession.intValue1 = 6;
    m_parameterInputSession.doubleValue1 = 360.0;
    m_parameterInputSession.point1 = m_parameterInputSession.centerPoint;
    m_parameterInputSession.boolValue = true;
    activateParameterInputSession();
    m_viewer->appendCommandMessage(QStringLiteral("环形阵列: 请依次输入项目数、填充角度、阵列中心和旋转方式"));
    return true;
}

bool CadController::beginMirrorSelected()
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    const QVector<CadItem*> selectedItems = m_viewer->selectedEntities();

    if (selectedItems.isEmpty())
    {
        return false;
    }

    const QVector3D centerPoint = geometryBoundsCenter(selectedItems);
    cancelDrawing();
    m_parameterInputSession.command = ParameterInputCommand::Mirror;
    m_parameterInputSession.selectedItems = selectedItems;
    m_parameterInputSession.point1 = QVector3D(centerPoint.x(), centerPoint.y() - 10.0f, 0.0f);
    m_parameterInputSession.point2 = QVector3D(centerPoint.x(), centerPoint.y() + 10.0f, 0.0f);
    m_parameterInputSession.boolValue = false;
    activateParameterInputSession();
    m_viewer->appendCommandMessage(QStringLiteral("镜像: 请指定镜像线两点，再输入是否删除原图元"));
    return true;
}

bool CadController::beginOffsetSelected()
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    CadItem* targetItem = m_viewer->selectedEntity();

    if (targetItem == nullptr)
    {
        const QVector<CadItem*> selectedItems = m_viewer->selectedEntities();

        if (selectedItems.isEmpty())
        {
            return false;
        }

        targetItem = selectedItems.front();
    }

    cancelDrawing();
    m_parameterInputSession.command = ParameterInputCommand::Offset;
    m_parameterInputSession.primaryItem = targetItem;
    m_parameterInputSession.doubleValue1 = 10.0;
    activateParameterInputSession();
    m_viewer->appendCommandMessage(QStringLiteral("偏移: 请在光标旁输入距离"));
    return true;
}

bool CadController::beginTrimSelected()
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    CadItem* targetItem = nullptr;
    CadItem* boundaryItem = nullptr;

    if (!resolveCurrentAndOtherSelectedItems(m_viewer, targetItem, boundaryItem))
    {
        return false;
    }

    cancelDrawing();
    m_parameterInputSession.command = ParameterInputCommand::Trim;
    m_parameterInputSession.primaryItem = targetItem;
    m_parameterInputSession.secondaryItem = boundaryItem;
    m_parameterInputSession.boolValue = false;
    activateParameterInputSession();
    m_viewer->appendCommandMessage(QStringLiteral("修剪: 请在光标旁输入要修剪的端点"));
    return true;
}

bool CadController::beginExtendSelected()
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    CadItem* targetItem = nullptr;
    CadItem* boundaryItem = nullptr;

    if (!resolveCurrentAndOtherSelectedItems(m_viewer, targetItem, boundaryItem))
    {
        return false;
    }

    cancelDrawing();
    m_parameterInputSession.command = ParameterInputCommand::Extend;
    m_parameterInputSession.primaryItem = targetItem;
    m_parameterInputSession.secondaryItem = boundaryItem;
    m_parameterInputSession.boolValue = false;
    activateParameterInputSession();
    m_viewer->appendCommandMessage(QStringLiteral("延申: 请在光标旁输入要延申的端点"));
    return true;
}

bool CadController::beginFilletSelected()
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    CadItem* secondItem = nullptr;
    CadItem* firstItem = nullptr;

    if (!resolveCurrentAndOtherSelectedItems(m_viewer, secondItem, firstItem))
    {
        return false;
    }

    cancelDrawing();
    m_parameterInputSession.command = ParameterInputCommand::Fillet;
    m_parameterInputSession.primaryItem = firstItem;
    m_parameterInputSession.secondaryItem = secondItem;
    m_parameterInputSession.doubleValue1 = 5.0;
    activateParameterInputSession();
    m_viewer->appendCommandMessage(QStringLiteral("圆角: 请在光标旁输入半径"));
    return true;
}

bool CadController::beginChamferSelected()
{
    if (m_viewer == nullptr)
    {
        return false;
    }

    CadItem* secondItem = nullptr;
    CadItem* firstItem = nullptr;

    if (!resolveCurrentAndOtherSelectedItems(m_viewer, secondItem, firstItem))
    {
        return false;
    }

    cancelDrawing();
    m_parameterInputSession.command = ParameterInputCommand::Chamfer;
    m_parameterInputSession.primaryItem = firstItem;
    m_parameterInputSession.secondaryItem = secondItem;
    m_parameterInputSession.doubleValue1 = 5.0;
    m_parameterInputSession.doubleValue2 = 5.0;
    activateParameterInputSession();
    m_viewer->appendCommandMessage(QStringLiteral("倒角: 请在光标旁输入两条边距离"));
    return true;
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
    clearDynamicCommandMode();
    resetParameterInputSession();
    resetPointDynamicInputSession();

    // 重置子模式
    resetSubModes();
    m_idleWindowSelectionTracking = false;
    m_idleWindowSelectionDragging = false;
    m_idleWindowSelectionAnchor = QPoint();
    m_idleWindowSelectionCurrent = QPoint();

    // 如果有视图，发送取消消息并刷新提示
    if (m_viewer != nullptr)
    {
        m_viewer->hideSelectionWindowPreview();

        if (hadActiveCommand)
        {
            m_viewer->appendCommandMessage(QStringLiteral("命令已取消"));
        }

        m_viewer->refreshCommandPrompt();
    }
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



bool CadController::deleteSelectedEntity()
{
    if (m_editer == nullptr || m_viewer == nullptr)
    {
        return false;
    }

    const QVector<CadItem*> selectedItems = m_viewer->selectedEntities();

    if (selectedItems.isEmpty())
    {
        return false;
    }

    int deletedCount = 0;

    for (CadItem* item : selectedItems)
    {
        if (item != nullptr && m_editer->deleteEntity(item))
        {
            ++deletedCount;
        }
    }

    if (deletedCount > 0)
    {
        m_viewer->appendCommandMessage
        (
            deletedCount > 1
                ? QStringLiteral("已删除 %1 个图元").arg(deletedCount)
                : QStringLiteral("已删除选中图元")
        );
        m_viewer->refreshCommandPrompt();
        return true;
    }

    return false;
}

bool CadController::changeSelectedEntityColor()
{
    if (m_editer == nullptr || m_viewer == nullptr)
    {
        return false;
    }

    const QVector<CadItem*> selectedItems = m_viewer->selectedEntities();

    if (selectedItems.isEmpty())
    {
        return true;
    }

    CadItem* selectedItem = m_viewer->selectedEntity();

    if (selectedItem == nullptr)
    {
        selectedItem = selectedItems.front();
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

    int changedCount = 0;

    for (CadItem* item : selectedItems)
    {
        if (item != nullptr && m_editer->changeEntityColor(item, color))
        {
            ++changedCount;
        }
    }

    if (changedCount > 0)
    {
        m_viewer->appendCommandMessage
        (
            changedCount > 1
                ? QStringLiteral("已修改 %1 个图元颜色").arg(changedCount)
                : QStringLiteral("已修改图元颜色")
        );
        m_viewer->refreshCommandPrompt();
        return true;
    }

    return false;
}
// 重置所有子模式
void CadController::resetSubModes()
{
    m_drawState.pointSubMode = PointDrawSubMode::Idle;
    m_drawState.lineSubMode = LineDrawSubMode::Idle;
    m_drawState.circleSubMode = CircleDrawSubMode::Idle;
    m_drawState.rectangleSubMode = RectangleDrawSubMode::Idle;
    m_drawState.polygonSubMode = PolygonDrawSubMode::Idle;
    m_drawState.arcSubMode = ArcDrawSubMode::Idle;
    m_drawState.ellipseSubMode = EllipseDrawSubMode::Idle;
    m_drawState.polylineSubMode = PolylineDrawSubMode::Idle;
    m_drawState.lwPolylineSubMode = LWPolylineDrawSubMode::Idle;
    m_drawState.moveSubMode = MoveEditSubMode::Idle;
    m_drawState.gripSubMode = GripEditSubMode::Idle;
    m_drawState.gripPointIndex = -1;
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
    case DrawType::Xline:
        m_drawState.lineSubMode = LineDrawSubMode::AwaitStartPoint;
        break;
    case DrawType::Rectangle:
        m_drawState.rectangleSubMode = RectangleDrawSubMode::AwaitFirstCorner;
        break;
    case DrawType::Polygon:
        m_drawState.polygonSubMode = PolygonDrawSubMode::AwaitCenter;
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
    case DrawType::Xline:
        if (m_drawState.lineSubMode == LineDrawSubMode::AwaitStartPoint)
        {
            m_drawState.lineSubMode = LineDrawSubMode::AwaitEndPoint;
        }
        else
        {
            m_drawState.lineSubMode = LineDrawSubMode::AwaitStartPoint;
        }
        break;
    case DrawType::Rectangle:
        if (m_drawState.rectangleSubMode == RectangleDrawSubMode::AwaitFirstCorner)
        {
            m_drawState.rectangleSubMode = RectangleDrawSubMode::AwaitSecondCorner;
        }
        else
        {
            m_drawState.rectangleSubMode = RectangleDrawSubMode::AwaitFirstCorner;
        }
        break;
    case DrawType::Polygon:
        if (m_drawState.polygonSubMode == PolygonDrawSubMode::AwaitCenter)
        {
            m_drawState.polygonSubMode = PolygonDrawSubMode::AwaitRadius;
        }
        else
        {
            m_drawState.polygonSubMode = PolygonDrawSubMode::AwaitCenter;
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

void CadController::syncCurrentPosWithCursor()
{
    if (m_viewer == nullptr || !m_drawState.hasActiveCommand())
    {
        return;
    }

    const QPoint localCursorPos = m_viewer->mapFromGlobal(QCursor::pos());

    if (m_viewer->rect().contains(localCursorPos))
    {
        m_drawState.currentScreenPos = localCursorPos;
    }

    QVector3D resolvedWorldPos = applyOrthoConstraint(currentWorldPos(m_drawState.currentScreenPos));

    if (isPointDynamicFieldModeActive())
    {
        resolvedWorldPos = applyPointDynamicFieldOverride(resolvedWorldPos, true);
    }

    m_drawState.currentPos = resolvedWorldPos;
}

