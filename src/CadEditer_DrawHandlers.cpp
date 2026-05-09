// CadEditer 实现文件
// 实现 CadEditer 模块，对应头文件中声明的主要行为和协作流程。
// 编辑器模块，负责绘图创建、实体修改以及 Undo/Redo 命令栈管理。
#include "pch.h"

#include "CadEditer.h"

#include "CadDocument.h"
#include "CadItem.h"
#include "DrawStateMachine.h"

#include <QSet>

#include <algorithm>
#include <cmath>
#include <limits>

#include "CadEditerWorkflowInternal.h"

using namespace CadEditerWorkflowInternal;

// 结束当前活动多段线命令
// @param drawState 当前绘图状态机
// @param closePolyline 是否闭合多段线
// @return 如果成功生成实体返回 true，否则返回 false
bool CadEditer::finishActivePolyline(DrawStateMachine& drawState, bool closePolyline)
{
    if (m_document == nullptr)
    {
        return false;
    }

    if (drawState.drawType != DrawType::Polyline && drawState.drawType != DrawType::LWPolyline)
    {
        return false;
    }

    if (drawState.commandPoints.size() < 2)
    {
        return false;
    }

    const bool lightweight = drawState.drawType == DrawType::LWPolyline;
    std::unique_ptr<DRW_Entity> entity = createPolylineEntity
    (
        drawState.commandPoints,
        drawState.commandBulges,
        drawState.drawingLayerName,
        drawState.drawingColor,
        drawState.drawingColorIndex,
        closePolyline,
        lightweight
    );

    if (!addEntity(std::move(entity)))
    {
        return false;
    }

    // 成功落库后清空暂存点列，并把状态机复位到等待第一点
    drawState.commandPoints.clear();
    drawState.commandBulges.clear();

    if (lightweight)
    {
        drawState.lwPolylineSubMode = LWPolylineDrawSubMode::AwaitFirstPoint;
        drawState.lwPolylineArcMode = false;
    }
    else
    {
        drawState.polylineSubMode = PolylineDrawSubMode::AwaitFirstPoint;
        drawState.polylineArcMode = false;
    }

    return true;
}

// 开始移动编辑命令
// @param drawState 当前绘图状态机
// @param item 待移动实体
// @return 如果命令成功进入活动状态返回 true，否则返回 false
bool CadEditer::beginMove(DrawStateMachine& drawState, CadItem* item)
{
    return beginMove(drawState, QVector<CadItem*>{ item });
}

bool CadEditer::beginMove(DrawStateMachine& drawState, const QVector<CadItem*>& items)
{
    if (m_document == nullptr || items.isEmpty())
    {
        return false;
    }

    QSet<CadItem*> dedup;
    QVector<CadItem*> validatedTargets;
    validatedTargets.reserve(items.size());

    for (CadItem* item : items)
    {
        if (item == nullptr || !m_document->containsEntity(item) || dedup.contains(item))
        {
            continue;
        }

        dedup.insert(item);
        validatedTargets.append(item);
    }

    if (validatedTargets.isEmpty())
    {
        return false;
    }

    drawState.commandPoints.clear();
    drawState.commandBulges.clear();
    drawState.isDrawing = false;
    drawState.drawType = DrawType::None;
    drawState.editType = EditType::Move;
    drawState.moveSubMode = MoveEditSubMode::AwaitBasePoint;
    drawState.gripSubMode = GripEditSubMode::Idle;
    drawState.gripPointIndex = -1;
    m_moveTargets = validatedTargets;
    m_moveTarget = m_moveTargets.front();
    m_gripTarget = nullptr;
    m_gripPointIndex = -1;
    return true;
}

bool CadEditer::beginGripEdit(DrawStateMachine& drawState, CadItem* item, const CadSelectionHandleInfo& handle)
{
    if (m_document == nullptr
        || item == nullptr
        || !m_document->containsEntity(item)
        || !handle.editable
        || handle.pointIndex < 0)
    {
        return false;
    }

    QVector3D currentPoint;

    if (!readEditableControlPoint(item, handle.pointIndex, currentPoint))
    {
        return false;
    }

    drawState.commandPoints = { currentPoint };
    drawState.commandBulges.clear();
    drawState.isDrawing = false;
    drawState.drawType = DrawType::None;
    drawState.editType = EditType::GripEdit;
    drawState.moveSubMode = MoveEditSubMode::Idle;
    drawState.gripSubMode = GripEditSubMode::AwaitTargetPoint;
    drawState.gripPointIndex = handle.pointIndex;
    m_moveTarget = nullptr;
    m_moveTargets.clear();
    m_gripTarget = item;
    m_gripPointIndex = handle.pointIndex;
    return true;
}

// 处理点绘制命令
bool CadEditer::handlePointDrawing
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    Q_UNUSED(currentState);

    if (previousState.pointSubMode != PointDrawSubMode::AwaitPosition)
    {
        return false;
    }

    return addEntity
    (
        createPointEntity
        (
            worldPos,
            previousState.drawingLayerName,
            previousState.drawingColor,
            previousState.drawingColorIndex
        )
    );
}

// 处理直线绘制命令
bool CadEditer::handleLineDrawing
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    if (previousState.lineSubMode == LineDrawSubMode::AwaitStartPoint)
    {
        // 第一次点击只记录起点
        currentState.commandPoints = { worldPos };
        return true;
    }

    if (previousState.lineSubMode == LineDrawSubMode::AwaitEndPoint && !currentState.commandPoints.isEmpty())
    {
        const QVector3D startPoint = currentState.commandPoints.front();

        if (!addEntity
        (
            createLineEntity
            (
                startPoint,
                worldPos,
                previousState.drawingLayerName,
                previousState.drawingColor,
                previousState.drawingColorIndex
            )
        ))
        {
            return false;
        }

        // 成功创建一段线后，把终点作为下一段的起点，支持连续折线式输入
        currentState.commandPoints = { worldPos };
        currentState.lineSubMode = LineDrawSubMode::AwaitEndPoint;
        return true;
    }

    return false;
}

bool CadEditer::handleXlineDrawing
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    if (previousState.lineSubMode == LineDrawSubMode::AwaitStartPoint)
    {
        currentState.commandPoints = { worldPos };
        return true;
    }

    if (previousState.lineSubMode == LineDrawSubMode::AwaitEndPoint && !currentState.commandPoints.isEmpty())
    {
        const QVector3D basePoint = currentState.commandPoints.front();

        if (!addEntity
        (
            createXlineEntity
            (
                basePoint,
                worldPos,
                previousState.drawingLayerName,
                previousState.drawingColor,
                previousState.drawingColorIndex
            )
        ))
        {
            return false;
        }

        currentState.commandPoints.clear();
        currentState.lineSubMode = LineDrawSubMode::AwaitStartPoint;
        return true;
    }

    return false;
}

bool CadEditer::handleRectangleDrawing
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    if (previousState.rectangleSubMode == RectangleDrawSubMode::AwaitFirstCorner)
    {
        currentState.commandPoints = { flattenToDrawingPlane(worldPos) };
        return true;
    }

    if (previousState.rectangleSubMode == RectangleDrawSubMode::AwaitSecondCorner && !currentState.commandPoints.isEmpty())
    {
        const QVector3D firstCorner = flattenToDrawingPlane(currentState.commandPoints.front());

        if (!addEntity
        (
            createRectangleEntity
            (
                firstCorner,
                worldPos,
                previousState.drawingLayerName,
                previousState.drawingColor,
                previousState.drawingColorIndex
            )
        ))
        {
            return false;
        }

        currentState.commandPoints.clear();
        currentState.rectangleSubMode = RectangleDrawSubMode::AwaitFirstCorner;
        return true;
    }

    return false;
}

bool CadEditer::handlePolygonDrawing
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    if (previousState.polygonSubMode == PolygonDrawSubMode::AwaitCenter)
    {
        currentState.commandPoints = { flattenToDrawingPlane(worldPos) };
        return true;
    }

    if (previousState.polygonSubMode == PolygonDrawSubMode::AwaitRadius && !currentState.commandPoints.isEmpty())
    {
        const QVector3D center = flattenToDrawingPlane(currentState.commandPoints.front());

        if (!addEntity
        (
            createPolygonEntity
            (
                center,
                worldPos,
                previousState.polygonSideCount,
                previousState.polygonCircumscribedAboutCircle,
                previousState.drawingLayerName,
                previousState.drawingColor,
                previousState.drawingColorIndex
            )
        ))
        {
            return false;
        }

        currentState.commandPoints.clear();
        currentState.polygonSubMode = PolygonDrawSubMode::AwaitCenter;
        return true;
    }

    return false;
}

// 处理圆绘制命令
bool CadEditer::handleCircleDrawing
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    if (previousState.circleSubMode == CircleDrawSubMode::AwaitCenter)
    {
        currentState.commandPoints = { worldPos };
        return true;
    }

    if (previousState.circleSubMode == CircleDrawSubMode::AwaitRadius && !currentState.commandPoints.isEmpty())
    {
        const QVector3D center = currentState.commandPoints.front();

        if (!addEntity
        (
            createCircleEntity
            (
                center,
                worldPos,
                previousState.drawingLayerName,
                previousState.drawingColor,
                previousState.drawingColorIndex
            )
        ))
        {
            return false;
        }

        currentState.commandPoints.clear();
        currentState.circleSubMode = CircleDrawSubMode::AwaitCenter;
        return true;
    }

    return false;
}

// 处理圆弧绘制命令
bool CadEditer::handleArcDrawing
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    switch (previousState.arcSubMode)
    {
    case ArcDrawSubMode::AwaitCenter:
        currentState.commandPoints = { worldPos };
        return true;

    case ArcDrawSubMode::AwaitRadius:
        if (currentState.commandPoints.isEmpty())
        {
            return false;
        }

        if (currentState.commandPoints.size() == 1)
        {
            currentState.commandPoints.append(worldPos);
        }
        else
        {
            currentState.commandPoints[1] = worldPos;
        }
        return true;

    case ArcDrawSubMode::AwaitStartAngle:
        return true;

    case ArcDrawSubMode::AwaitEndAngle:
        if (currentState.commandPoints.size() < 2)
        {
            return false;
        }

        // 圆弧绘制按“圆心 -> 起点/半径 -> 终点”三步完成；Ctrl 切换补弧。
        if (!addEntity
        (
            createArcEntity
            (
                currentState.commandPoints[0],
                currentState.commandPoints[1],
                currentState.commandPoints[1],
                worldPos,
                previousState.drawingLayerName,
                previousState.drawingColor,
                previousState.drawingColorIndex,
                (previousState.keyboardModifiers & Qt::ControlModifier) != 0
            )
        ))
        {
            return false;
        }

        currentState.commandPoints.clear();
        currentState.arcSubMode = ArcDrawSubMode::AwaitCenter;
        return true;

    default:
        break;
    }

    return false;
}

// 处理椭圆绘制命令
bool CadEditer::handleEllipseDrawing
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    switch (previousState.ellipseSubMode)
    {
    case EllipseDrawSubMode::AwaitCenter:
        currentState.commandPoints = { worldPos };
        return true;

    case EllipseDrawSubMode::AwaitMajorAxis:
        if (currentState.commandPoints.isEmpty())
        {
            return false;
        }

        if (currentState.commandPoints.size() == 1)
        {
            currentState.commandPoints.append(worldPos);
        }
        else
        {
            currentState.commandPoints[1] = worldPos;
        }
        return true;

    case EllipseDrawSubMode::AwaitMinorAxis:
        if (currentState.commandPoints.size() < 2)
        {
            return false;
        }

        if (!addEntity
        (
            createEllipseEntity
            (
                currentState.commandPoints[0],
                currentState.commandPoints[1],
                worldPos,
                previousState.drawingLayerName,
                previousState.drawingColor,
                previousState.drawingColorIndex
            )
        ))
        {
            return false;
        }

        currentState.commandPoints.clear();
        currentState.ellipseSubMode = EllipseDrawSubMode::AwaitCenter;
        return true;

    default:
        break;
    }

    return false;
}

// 处理多段线/轻量多段线绘制命令
bool CadEditer::handlePolylineDrawing
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos,
    bool lightweight
)
{
    const QVector3D planarPoint = flattenToDrawingPlane(worldPos);
    const bool arcMode = lightweight ? currentState.lwPolylineArcMode : currentState.polylineArcMode;

    if ((lightweight && previousState.lwPolylineSubMode == LWPolylineDrawSubMode::AwaitFirstPoint)
        || (!lightweight && previousState.polylineSubMode == PolylineDrawSubMode::AwaitFirstPoint))
    {
        // 第一击只建立起始点，并切换到后续线段/圆弧段输入状态
        currentState.commandPoints = { planarPoint };
        currentState.commandBulges.clear();

        if (lightweight)
        {
            currentState.lwPolylineSubMode = arcMode ? LWPolylineDrawSubMode::AwaitArcEndPoint : LWPolylineDrawSubMode::AwaitLineEndPoint;
        }
        else
        {
            currentState.polylineSubMode = arcMode ? PolylineDrawSubMode::AwaitArcEndPoint : PolylineDrawSubMode::AwaitLineEndPoint;
        }

        return true;
    }

    if (currentState.commandPoints.isEmpty())
    {
        return false;
    }

    const QVector3D lastPoint = flattenToDrawingPlane(currentState.commandPoints.back());

    if ((planarPoint - lastPoint).lengthSquared() <= kGeometryEpsilon)
    {
        return false;
    }

    if ((lightweight && previousState.lwPolylineSubMode == LWPolylineDrawSubMode::AwaitArcEndPoint)
        || (!lightweight && previousState.polylineSubMode == PolylineDrawSubMode::AwaitArcEndPoint))
    {
        // 圆弧续接依赖上一段末切向，按切向与新终点实时反推 bulge
        const QVector3D tangentDirection = polylineEndTangent(currentState.commandPoints, currentState.commandBulges);

        if (tangentDirection.lengthSquared() <= kGeometryEpsilon)
        {
            return false;
        }

        const double bulge = bulgeFromTangent(lastPoint, tangentDirection, planarPoint);

        if (!std::isfinite(bulge))
        {
            return false;
        }

        currentState.commandBulges.append(bulge);
        currentState.commandPoints.append(planarPoint);

        if (lightweight)
        {
            currentState.lwPolylineSubMode = LWPolylineDrawSubMode::AwaitArcEndPoint;
        }
        else
        {
            currentState.polylineSubMode = PolylineDrawSubMode::AwaitArcEndPoint;
        }

        return true;
    }

    // 直线段直接以 bulge=0 追加
    currentState.commandBulges.append(0.0);
    currentState.commandPoints.append(planarPoint);

    if (lightweight)
    {
        currentState.lwPolylineSubMode = LWPolylineDrawSubMode::AwaitLineEndPoint;
    }
    else
    {
        currentState.polylineSubMode = PolylineDrawSubMode::AwaitLineEndPoint;
    }

    return true;
}



