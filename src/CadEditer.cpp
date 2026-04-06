#include "pch.h"

#include "CadEditer.h"

#include "CadDocument.h"
#include "CadItem.h"
#include "DrawStateMachine.h"

#include <cmath>
#include <limits>

namespace
{
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 6.28318530717958647692;
    constexpr double kGeometryEpsilon = 1.0e-9;

    QVector3D flattenToDrawingPlane(const QVector3D& point)
    {
        return QVector3D(point.x(), point.y(), 0.0f);
    }

    void translateCoord(DRW_Coord& point, const QVector3D& delta)
    {
        point.x += delta.x();
        point.y += delta.y();
        point.z += delta.z();
    }

    void translateEntity(DRW_Entity* entity, const QVector3D& delta)
    {
        if (entity == nullptr)
        {
            return;
        }

        switch (entity->eType)
        {
        case DRW::ETYPE::POINT:
        {
            translateCoord(static_cast<DRW_Point*>(entity)->basePoint, delta);
            break;
        }
        case DRW::ETYPE::LINE:
        {
            DRW_Line* line = static_cast<DRW_Line*>(entity);
            translateCoord(line->basePoint, delta);
            translateCoord(line->secPoint, delta);
            break;
        }
        case DRW::ETYPE::CIRCLE:
        {
            translateCoord(static_cast<DRW_Circle*>(entity)->basePoint, delta);
            break;
        }
        case DRW::ETYPE::ARC:
        {
            translateCoord(static_cast<DRW_Arc*>(entity)->basePoint, delta);
            break;
        }
        case DRW::ETYPE::ELLIPSE:
        {
            translateCoord(static_cast<DRW_Ellipse*>(entity)->basePoint, delta);
            break;
        }
        case DRW::ETYPE::LWPOLYLINE:
        {
            DRW_LWPolyline* polyline = static_cast<DRW_LWPolyline*>(entity);
            polyline->elevation += delta.z();

            for (const std::shared_ptr<DRW_Vertex2D>& vertex : polyline->vertlist)
            {
                vertex->x += delta.x();
                vertex->y += delta.y();
            }
            break;
        }
        case DRW::ETYPE::POLYLINE:
        {
            DRW_Polyline* polyline = static_cast<DRW_Polyline*>(entity);
            translateCoord(polyline->basePoint, delta);

            for (const std::shared_ptr<DRW_Vertex>& vertex : polyline->vertlist)
            {
                translateCoord(vertex->basePoint, delta);
            }
            break;
        }
        default:
            break;
        }
    }

    int colorToTrueColor(const QColor& color)
    {
        return (color.red() << 16) | (color.green() << 8) | color.blue();
    }

    void applyEntityColor(DRW_Entity* entity, const QColor& color, int colorIndex)
    {
        if (entity == nullptr)
        {
            return;
        }

        if (colorIndex >= 0)
        {
            entity->color = colorIndex;
            entity->color24 = -1;
            return;
        }

        entity->color = DRW::ColorByLayer;
        entity->color24 = colorToTrueColor(color);
    }

    double radiusFromPoints(const QVector3D& center, const QVector3D& point)
    {
        return (point - center).length();
    }

    QVector3D projectToCircle(const QVector3D& center, const QVector3D& radiusPoint, const QVector3D& pickPoint)
    {
        const double radius = radiusFromPoints(center, radiusPoint);

        if (radius <= kGeometryEpsilon)
        {
            return radiusPoint;
        }

        QVector3D direction = pickPoint - center;

        if (direction.lengthSquared() <= kGeometryEpsilon)
        {
            direction = radiusPoint - center;
        }

        if (direction.lengthSquared() <= kGeometryEpsilon)
        {
            return radiusPoint;
        }

        direction.normalize();
        return center + direction * static_cast<float>(radius);
    }

    QVector3D normalizedOrZero(const QVector3D& vector)
    {
        if (vector.lengthSquared() <= kGeometryEpsilon)
        {
            return QVector3D();
        }

        QVector3D normalized = vector;
        normalized.normalize();
        return normalized;
    }

    QVector3D bulgeArcCenter(const QVector3D& startPoint, const QVector3D& endPoint, double bulge, bool* valid = nullptr)
    {
        const QVector3D chord = endPoint - startPoint;
        const double chordLength = chord.length();

        if (valid != nullptr)
        {
            *valid = false;
        }

        if (chordLength <= kGeometryEpsilon || std::abs(bulge) <= kGeometryEpsilon)
        {
            return QVector3D();
        }

        const QVector3D midpoint = (startPoint + endPoint) * 0.5f;
        const double centerOffset = chordLength * (1.0 / bulge - bulge) * 0.25;
        const QVector3D leftNormal
        (
            static_cast<float>(-chord.y() / chordLength),
            static_cast<float>(chord.x() / chordLength),
            0.0f
        );

        if (valid != nullptr)
        {
            *valid = true;
        }

        return midpoint + leftNormal * static_cast<float>(centerOffset);
    }

    QVector3D polylineEndTangent
    (
        const QVector<QVector3D>& points,
        const QVector<double>& bulges
    )
    {
        if (points.size() < 2)
        {
            return QVector3D();
        }

        const QVector3D startPoint = flattenToDrawingPlane(points[points.size() - 2]);
        const QVector3D endPoint = flattenToDrawingPlane(points.back());
        const double bulge = bulges.size() >= points.size() - 1 ? bulges[points.size() - 2] : 0.0;

        if (std::abs(bulge) <= kGeometryEpsilon)
        {
            return normalizedOrZero(endPoint - startPoint);
        }

        bool hasCenter = false;
        const QVector3D center = bulgeArcCenter(startPoint, endPoint, bulge, &hasCenter);

        if (!hasCenter)
        {
            return normalizedOrZero(endPoint - startPoint);
        }

        const QVector3D radiusVector = endPoint - center;
        QVector3D tangent;

        if (bulge > 0.0)
        {
            tangent = QVector3D(-radiusVector.y(), radiusVector.x(), 0.0f);
        }
        else
        {
            tangent = QVector3D(radiusVector.y(), -radiusVector.x(), 0.0f);
        }

        return normalizedOrZero(tangent);
    }

    double bulgeFromTangent(const QVector3D& startPoint, const QVector3D& tangentDirection, const QVector3D& endPoint)
    {
        const QVector3D planarStartPoint = flattenToDrawingPlane(startPoint);
        const QVector3D planarEndPoint = flattenToDrawingPlane(endPoint);
        const QVector3D planarTangent = normalizedOrZero(QVector3D(tangentDirection.x(), tangentDirection.y(), 0.0f));
        const QVector3D chordVector = planarEndPoint - planarStartPoint;

        if (planarTangent.lengthSquared() <= kGeometryEpsilon || chordVector.lengthSquared() <= kGeometryEpsilon)
        {
            return 0.0;
        }

        const double dotValue = QVector3D::dotProduct(planarTangent, chordVector);
        const double crossValue = planarTangent.x() * chordVector.y() - planarTangent.y() * chordVector.x();
        const double alpha = std::atan2(crossValue, dotValue);

        if (std::abs(std::abs(alpha) - kPi) <= 1.0e-6)
        {
            return std::numeric_limits<double>::infinity();
        }

        return std::tan(alpha * 0.5);
    }

    std::unique_ptr<DRW_Entity> createPointEntity(const QVector3D& position, const QColor& color)
    {
        const QVector3D planarPosition = flattenToDrawingPlane(position);
        auto entity = std::make_unique<DRW_Point>();
        entity->basePoint.x = planarPosition.x();
        entity->basePoint.y = planarPosition.y();
        entity->basePoint.z = 0.0;
        applyEntityColor(entity.get(), color, -1);
        return entity;
    }

    std::unique_ptr<DRW_Entity> createLineEntity(const QVector3D& startPoint, const QVector3D& endPoint, const QColor& color)
    {
        const QVector3D planarStartPoint = flattenToDrawingPlane(startPoint);
        const QVector3D planarEndPoint = flattenToDrawingPlane(endPoint);

        if ((planarEndPoint - planarStartPoint).lengthSquared() <= kGeometryEpsilon)
        {
            return nullptr;
        }

        auto entity = std::make_unique<DRW_Line>();
        entity->basePoint.x = planarStartPoint.x();
        entity->basePoint.y = planarStartPoint.y();
        entity->basePoint.z = 0.0;
        entity->secPoint.x = planarEndPoint.x();
        entity->secPoint.y = planarEndPoint.y();
        entity->secPoint.z = 0.0;
        applyEntityColor(entity.get(), color, -1);
        return entity;
    }

    std::unique_ptr<DRW_Entity> createCircleEntity(const QVector3D& center, const QVector3D& radiusPoint, const QColor& color)
    {
        const QVector3D planarCenter = flattenToDrawingPlane(center);
        const QVector3D planarRadiusPoint = flattenToDrawingPlane(radiusPoint);
        const double radius = radiusFromPoints(planarCenter, planarRadiusPoint);

        if (radius <= kGeometryEpsilon)
        {
            return nullptr;
        }

        auto entity = std::make_unique<DRW_Circle>();
        entity->basePoint.x = planarCenter.x();
        entity->basePoint.y = planarCenter.y();
        entity->basePoint.z = 0.0;
        entity->radious = radius;
        entity->extPoint.x = 0.0;
        entity->extPoint.y = 0.0;
        entity->extPoint.z = 1.0;
        applyEntityColor(entity.get(), color, -1);
        return entity;
    }

    std::unique_ptr<DRW_Entity> createArcEntity
    (
        const QVector3D& center,
        const QVector3D& radiusPoint,
        const QVector3D& startAnglePoint,
        const QVector3D& endAnglePoint,
        const QColor& color
    )
    {
        const QVector3D planarCenter = flattenToDrawingPlane(center);
        const QVector3D planarRadiusPoint = flattenToDrawingPlane(radiusPoint);
        const QVector3D planarStartAnglePoint = flattenToDrawingPlane(startAnglePoint);
        const QVector3D planarEndAnglePoint = flattenToDrawingPlane(endAnglePoint);
        const double radius = radiusFromPoints(planarCenter, planarRadiusPoint);

        if (radius <= kGeometryEpsilon)
        {
            return nullptr;
        }

        const QVector3D startPoint = projectToCircle(planarCenter, planarRadiusPoint, planarStartAnglePoint);
        const QVector3D endPoint = projectToCircle(planarCenter, planarRadiusPoint, planarEndAnglePoint);

        auto entity = std::make_unique<DRW_Arc>();
        entity->basePoint.x = planarCenter.x();
        entity->basePoint.y = planarCenter.y();
        entity->basePoint.z = 0.0;
        entity->radious = radius;
        entity->staangle = std::atan2(startPoint.y() - planarCenter.y(), startPoint.x() - planarCenter.x());
        entity->endangle = std::atan2(endPoint.y() - planarCenter.y(), endPoint.x() - planarCenter.x());
        entity->extPoint.x = 0.0;
        entity->extPoint.y = 0.0;
        entity->extPoint.z = 1.0;

        if (std::abs(entity->endangle - entity->staangle) <= kGeometryEpsilon)
        {
            entity->endangle = entity->staangle + kTwoPi;
        }

        applyEntityColor(entity.get(), color, -1);
        return entity;
    }

    std::unique_ptr<DRW_Entity> createEllipseEntity
    (
        const QVector3D& center,
        const QVector3D& majorAxisPoint,
        const QVector3D& ratioPoint,
        const QColor& color
    )
    {
        const QVector3D planarCenter = flattenToDrawingPlane(center);
        const QVector3D planarMajorAxisPoint = flattenToDrawingPlane(majorAxisPoint);
        const QVector3D planarRatioPoint = flattenToDrawingPlane(ratioPoint);
        const QVector3D majorAxis = planarMajorAxisPoint - planarCenter;
        const double majorLength = majorAxis.length();

        if (majorLength <= kGeometryEpsilon)
        {
            return nullptr;
        }

        const QVector3D toRatioPoint = planarRatioPoint - planarCenter;
        const double projectedLength = QVector3D::dotProduct(toRatioPoint, majorAxis) / majorLength;
        const double minorSquared = std::max(0.0, static_cast<double>(toRatioPoint.lengthSquared()) - projectedLength * projectedLength);
        const double minorLength = std::sqrt(minorSquared);

        if (minorLength <= kGeometryEpsilon)
        {
            return nullptr;
        }

        auto entity = std::make_unique<DRW_Ellipse>();
        entity->basePoint.x = planarCenter.x();
        entity->basePoint.y = planarCenter.y();
        entity->basePoint.z = 0.0;
        entity->secPoint.x = majorAxis.x();
        entity->secPoint.y = majorAxis.y();
        entity->secPoint.z = 0.0;
        entity->ratio = minorLength / majorLength;
        entity->staparam = 0.0;
        entity->endparam = 2.0 * kPi;
        entity->extPoint.x = 0.0;
        entity->extPoint.y = 0.0;
        entity->extPoint.z = 1.0;
        applyEntityColor(entity.get(), color, -1);
        return entity;
    }

    std::unique_ptr<DRW_Entity> createPolylineEntity
    (
        const QVector<QVector3D>& points,
        const QVector<double>& bulges,
        const QColor& color,
        bool closePolyline,
        bool lightweight
    )
    {
        if (points.size() < 2)
        {
            return nullptr;
        }

        if (lightweight)
        {
            auto entity = std::make_unique<DRW_LWPolyline>();
            entity->flags = closePolyline ? 1 : 0;
            entity->elevation = 0.0;

            for (const QVector3D& point : points)
            {
                const QVector3D planarPoint = flattenToDrawingPlane(point);
                std::shared_ptr<DRW_Vertex2D> vertex = std::make_shared<DRW_Vertex2D>();
                vertex->x = planarPoint.x();
                vertex->y = planarPoint.y();
                entity->vertlist.push_back(vertex);
            }

            for (int i = 0; i < entity->vertlist.size(); ++i)
            {
                entity->vertlist[static_cast<size_t>(i)]->bulge = i < bulges.size() ? bulges[i] : 0.0;
            }

            entity->vertexnum = static_cast<int>(entity->vertlist.size());
            applyEntityColor(entity.get(), color, -1);
            return entity;
        }

        auto entity = std::make_unique<DRW_Polyline>();
        entity->flags = closePolyline ? 1 : 0;

        for (const QVector3D& point : points)
        {
            const QVector3D planarPoint = flattenToDrawingPlane(point);
            std::shared_ptr<DRW_Vertex> vertex = std::make_shared<DRW_Vertex>();
            vertex->basePoint.x = planarPoint.x();
            vertex->basePoint.y = planarPoint.y();
            vertex->basePoint.z = 0.0;
            entity->vertlist.push_back(vertex);
        }

        for (int i = 0; i < entity->vertlist.size(); ++i)
        {
            entity->vertlist[static_cast<size_t>(i)]->bulge = i < bulges.size() ? bulges[i] : 0.0;
        }

        entity->vertexcount = static_cast<int>(entity->vertlist.size());
        applyEntityColor(entity.get(), color, -1);
        return entity;
    }
}

class AddEntityCommand final : public CadEditer::EditCommand
{
public:
    AddEntityCommand(CadDocument* document, std::unique_ptr<DRW_Entity> entity)
        : m_document(document)
        , m_entity(std::move(entity))
    {
    }

    bool execute() override
    {
        if (m_document == nullptr || m_entity == nullptr)
        {
            return false;
        }

        if (m_item == nullptr)
        {
            m_item = CadDocument::createCadItemForEntity(m_entity.get());
        }

        if (m_item == nullptr)
        {
            return false;
        }

        m_itemPtr = m_item.get();
        return m_document->appendEntity(std::move(m_entity), std::move(m_item)) != nullptr;
    }

    bool undo() override
    {
        if (m_document == nullptr || m_itemPtr == nullptr)
        {
            return false;
        }

        auto [entity, item] = m_document->takeEntity(m_itemPtr);

        if (entity == nullptr || item == nullptr)
        {
            return false;
        }

        m_entity = std::move(entity);
        m_item = std::move(item);
        m_itemPtr = m_item.get();
        return true;
    }

private:
    CadDocument* m_document = nullptr;
    std::unique_ptr<DRW_Entity> m_entity;
    std::unique_ptr<CadItem> m_item;
    CadItem* m_itemPtr = nullptr;
};

class DeleteEntityCommand final : public CadEditer::EditCommand
{
public:
    DeleteEntityCommand(CadDocument* document, CadItem* item)
        : m_document(document)
        , m_itemPtr(item)
    {
    }

    bool execute() override
    {
        if (m_document == nullptr || m_itemPtr == nullptr)
        {
            return false;
        }

        auto [entity, item] = m_document->takeEntity(m_itemPtr);

        if (entity == nullptr || item == nullptr)
        {
            return false;
        }

        m_entity = std::move(entity);
        m_item = std::move(item);
        m_itemPtr = m_item.get();
        return true;
    }

    bool undo() override
    {
        if (m_document == nullptr || m_entity == nullptr || m_item == nullptr)
        {
            return false;
        }

        m_itemPtr = m_item.get();
        return m_document->appendEntity(std::move(m_entity), std::move(m_item)) != nullptr;
    }

private:
    CadDocument* m_document = nullptr;
    std::unique_ptr<DRW_Entity> m_entity;
    std::unique_ptr<CadItem> m_item;
    CadItem* m_itemPtr = nullptr;
};

class MoveEntityCommand final : public CadEditer::EditCommand
{
public:
    MoveEntityCommand(CadDocument* document, CadItem* item, const QVector3D& delta)
        : m_document(document)
        , m_item(item)
        , m_delta(delta)
    {
    }

    bool execute() override
    {
        return applyDelta(m_delta);
    }

    bool undo() override
    {
        return applyDelta(-m_delta);
    }

private:
    bool applyDelta(const QVector3D& delta)
    {
        if (m_document == nullptr || m_item == nullptr || !m_document->containsEntity(m_item))
        {
            return false;
        }

        translateEntity(m_item->m_nativeEntity, delta);
        return m_document->refreshEntity(m_item);
    }

private:
    CadDocument* m_document = nullptr;
    CadItem* m_item = nullptr;
    QVector3D m_delta;
};

class ChangeColorCommand final : public CadEditer::EditCommand
{
public:
    ChangeColorCommand(CadDocument* document, CadItem* item, const QColor& color, int colorIndex)
        : m_document(document)
        , m_item(item)
        , m_newColor(color)
        , m_newColorIndex(colorIndex)
    {
        if (m_item != nullptr && m_item->m_nativeEntity != nullptr)
        {
            m_oldColorIndex = m_item->m_nativeEntity->color;
            m_oldTrueColor = m_item->m_nativeEntity->color24;
        }
    }

    bool execute() override
    {
        return apply(m_newColor, m_newColorIndex);
    }

    bool undo() override
    {
        return apply(m_newColor, m_oldColorIndex, m_oldTrueColor);
    }

private:
    bool apply(const QColor& color, int colorIndex)
    {
        return apply(color, colorIndex, colorIndex >= 0 ? -1 : colorToTrueColor(color));
    }

    bool apply(const QColor& color, int colorIndex, int trueColor)
    {
        if (m_document == nullptr || m_item == nullptr || m_item->m_nativeEntity == nullptr || !m_document->containsEntity(m_item))
        {
            return false;
        }

        if (colorIndex >= 0)
        {
            m_item->m_nativeEntity->color = colorIndex;
            m_item->m_nativeEntity->color24 = -1;
        }
        else
        {
            m_item->m_nativeEntity->color = DRW::ColorByLayer;
            m_item->m_nativeEntity->color24 = trueColor;
        }

        Q_UNUSED(color);
        return m_document->refreshEntity(m_item);
    }

private:
    CadDocument* m_document = nullptr;
    CadItem* m_item = nullptr;
    QColor m_newColor;
    int m_newColorIndex = -1;
    int m_oldColorIndex = DRW::ColorByLayer;
    int m_oldTrueColor = -1;
};

CadEditer::~CadEditer() = default;

void CadEditer::setDocument(CadDocument* document)
{
    if (m_document == document)
    {
        return;
    }

    clearHistory();
    m_document = document;
}

void CadEditer::clearHistory()
{
    m_undoStack.clear();
    m_redoStack.clear();
    cancelTransientCommand();
}

void CadEditer::cancelTransientCommand()
{
    m_moveTarget = nullptr;
}

bool CadEditer::canUndo() const
{
    return !m_undoStack.empty();
}

bool CadEditer::canRedo() const
{
    return !m_redoStack.empty();
}

bool CadEditer::undo()
{
    if (m_undoStack.empty())
    {
        return false;
    }

    std::unique_ptr<EditCommand> command = std::move(m_undoStack.back());
    m_undoStack.pop_back();

    if (!command->undo())
    {
        return false;
    }

    m_redoStack.push_back(std::move(command));
    return true;
}

bool CadEditer::redo()
{
    if (m_redoStack.empty())
    {
        return false;
    }

    std::unique_ptr<EditCommand> command = std::move(m_redoStack.back());
    m_redoStack.pop_back();

    if (!command->execute())
    {
        return false;
    }

    m_undoStack.push_back(std::move(command));
    return true;
}

bool CadEditer::handleLeftPress
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    if (m_document == nullptr)
    {
        return false;
    }

    if (currentState.editType == EditType::Move || previousState.editType == EditType::Move)
    {
        return handleMoveEditing(previousState, currentState, worldPos);
    }

    if (!currentState.isDrawing && !previousState.isDrawing)
    {
        return false;
    }

    switch (previousState.drawType)
    {
    case DrawType::Point:
        return handlePointDrawing(previousState, currentState, worldPos);
    case DrawType::Line:
        return handleLineDrawing(previousState, currentState, worldPos);
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
        drawState.drawingColor,
        closePolyline,
        lightweight
    );

    if (!addEntity(std::move(entity)))
    {
        return false;
    }

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

bool CadEditer::beginMove(DrawStateMachine& drawState, CadItem* item)
{
    if (m_document == nullptr || item == nullptr || !m_document->containsEntity(item))
    {
        return false;
    }

    drawState.commandPoints.clear();
    drawState.commandBulges.clear();
    drawState.isDrawing = false;
    drawState.drawType = DrawType::None;
    drawState.editType = EditType::Move;
    drawState.moveSubMode = MoveEditSubMode::AwaitBasePoint;
    m_moveTarget = item;
    return true;
}

bool CadEditer::deleteEntity(CadItem* item)
{
    if (m_document == nullptr || item == nullptr || !m_document->containsEntity(item))
    {
        return false;
    }

    if (item == m_moveTarget)
    {
        m_moveTarget = nullptr;
    }

    return executeCommand(std::make_unique<DeleteEntityCommand>(m_document, item));
}

bool CadEditer::changeEntityColor(CadItem* item, const QColor& color, int colorIndex)
{
    if (m_document == nullptr || item == nullptr || !m_document->containsEntity(item) || !color.isValid())
    {
        return false;
    }

    return executeCommand(std::make_unique<ChangeColorCommand>(m_document, item, color, colorIndex));
}

bool CadEditer::executeCommand(std::unique_ptr<EditCommand> command)
{
    if (command == nullptr)
    {
        return false;
    }

    if (!command->execute())
    {
        return false;
    }

    m_redoStack.clear();
    m_undoStack.push_back(std::move(command));
    return true;
}

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

    return addEntity(createPointEntity(worldPos, previousState.drawingColor));
}

bool CadEditer::handleLineDrawing
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
        const QVector3D startPoint = currentState.commandPoints.front();

        if (!addEntity(createLineEntity(startPoint, worldPos, previousState.drawingColor)))
        {
            return false;
        }

        currentState.commandPoints = { worldPos };
        currentState.lineSubMode = LineDrawSubMode::AwaitEndPoint;
        return true;
    }

    return false;
}

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

        if (!addEntity(createCircleEntity(center, worldPos, previousState.drawingColor)))
        {
            return false;
        }

        currentState.commandPoints.clear();
        currentState.circleSubMode = CircleDrawSubMode::AwaitCenter;
        return true;
    }

    return false;
}

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
        if (currentState.commandPoints.size() < 2)
        {
            return false;
        }

        if (currentState.commandPoints.size() == 2)
        {
            currentState.commandPoints.append(worldPos);
        }
        else
        {
            currentState.commandPoints[2] = worldPos;
        }
        return true;

    case ArcDrawSubMode::AwaitEndAngle:
        if (currentState.commandPoints.size() < 3)
        {
            return false;
        }

        if (!addEntity
        (
            createArcEntity
            (
                currentState.commandPoints[0],
                currentState.commandPoints[1],
                currentState.commandPoints[2],
                worldPos,
                previousState.drawingColor
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
                previousState.drawingColor
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

bool CadEditer::handleMoveEditing
(
    const DrawStateMachine& previousState,
    DrawStateMachine& currentState,
    const QVector3D& worldPos
)
{
    if (m_moveTarget == nullptr || m_document == nullptr || !m_document->containsEntity(m_moveTarget))
    {
        currentState.commandPoints.clear();
        currentState.editType = EditType::None;
        currentState.moveSubMode = MoveEditSubMode::Idle;
        m_moveTarget = nullptr;
        return false;
    }

    if (previousState.moveSubMode == MoveEditSubMode::AwaitBasePoint)
    {
        currentState.commandPoints = { worldPos };
        return true;
    }

    if (previousState.moveSubMode == MoveEditSubMode::AwaitTargetPoint && !currentState.commandPoints.isEmpty())
    {
        const QVector3D basePoint = currentState.commandPoints.front();
        const QVector3D delta = worldPos - basePoint;

        if (delta.lengthSquared() > kGeometryEpsilon)
        {
            if (!executeCommand(std::make_unique<MoveEntityCommand>(m_document, m_moveTarget, delta)))
            {
                return false;
            }
        }

        currentState.commandPoints.clear();
        currentState.editType = EditType::None;
        currentState.moveSubMode = MoveEditSubMode::Idle;
        m_moveTarget = nullptr;
        return true;
    }

    return false;
}

bool CadEditer::addEntity(std::unique_ptr<DRW_Entity> entity)
{
    if (entity == nullptr)
    {
        return false;
    }

    return executeCommand(std::make_unique<AddEntityCommand>(m_document, std::move(entity)));
}
