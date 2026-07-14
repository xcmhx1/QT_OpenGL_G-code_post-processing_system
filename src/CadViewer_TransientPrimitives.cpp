// CadViewer 临时图元构建实现
#include "pch.h"

#include "CadViewer.h"

#include "CadDocument.h"
#include "CadItem.h"
#include "CadPreviewBuilder.h"
#include "CadProcessVisualUtils.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

namespace
{
    QVector3D buildPerpendicularDirection(const QVector3D& direction)
    {
        QVector3D normalizedDirection = direction;

        if (normalizedDirection.lengthSquared() <= 1.0e-6f)
        {
            return QVector3D(1.0f, 0.0f, 0.0f);
        }

        normalizedDirection.normalize();

        QVector3D referenceAxis =
            std::abs(QVector3D::dotProduct(normalizedDirection, QVector3D(0.0f, 0.0f, 1.0f))) < 0.95f
            ? QVector3D(0.0f, 0.0f, 1.0f)
            : QVector3D(1.0f, 0.0f, 0.0f);

        QVector3D perpendicular = QVector3D::crossProduct(normalizedDirection, referenceAxis);

        if (perpendicular.lengthSquared() <= 1.0e-6f)
        {
            referenceAxis = QVector3D(0.0f, 1.0f, 0.0f);
            perpendicular = QVector3D::crossProduct(normalizedDirection, referenceAxis);
        }

        if (perpendicular.lengthSquared() <= 1.0e-6f)
        {
            return QVector3D(1.0f, 0.0f, 0.0f);
        }

        perpendicular.normalize();
        return perpendicular;
    }

    QVector<QVector3D> buildTriangleVertices(const QVector3D& center, QVector3D direction, float size)
    {
        if (direction.lengthSquared() <= 1.0e-6f)
        {
            direction = QVector3D(0.0f, 1.0f, 0.0f);
        }
        else
        {
            direction.normalize();
        }

        const QVector3D perpendicular = buildPerpendicularDirection(direction);

        const QVector3D tip = center + direction * size;
        const QVector3D baseCenter = center - direction * (size * 0.8f);
        const QVector3D left = baseCenter + perpendicular * (size * 0.7f);
        const QVector3D right = baseCenter - perpendicular * (size * 0.7f);
        return { tip, left, right };
    }

}

std::vector<TransientPrimitive> CadViewer::buildTransientPrimitives() const
{
    return CadPreviewBuilder::buildTransientPrimitives(m_controller.drawState(), selectedEntity(), selectedEntities());
}

std::vector<TransientPrimitive> CadViewer::buildProcessArrowPrimitives() const
{
    if (!m_processVisualsVisible || !m_processDirectionVisible)
    {
        return {};
    }

    CadDocument* scene = m_sceneCoordinator.document();

    if (scene == nullptr)
    {
        return {};
    }

    std::vector<TransientPrimitive> primitives;
    primitives.reserve(scene->m_entities.size());

    const float pixelScale = std::max(pixelToWorldScale(), 1.0e-4f);
    const float headLength = pixelScale * 13.0f;
    const float headHalfWidth = pixelScale * 5.0f;
    const QVector3D viewForward = m_camera.forwardDirection().lengthSquared() > 1.0e-6f
        ? m_camera.forwardDirection().normalized()
        : QVector3D(0.0f, 0.0f, -1.0f);

    for (const std::unique_ptr<CadItem>& entity : scene->m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        const auto* presentation = m_processPresentation != nullptr
            ? m_processPresentation->find(entity->m_entityId) : nullptr;
        if (presentation == nullptr || presentation->excluded) continue;
        const CadProcessVisualInfo info = buildProcessVisualInfo(entity.get(), presentation);

        if (!info.valid || info.direction.lengthSquared() <= 1.0e-6f)
        {
            continue;
        }

        QVector3D perpendicular = QVector3D::crossProduct(viewForward, info.direction);

        if (perpendicular.lengthSquared() <= 1.0e-6f)
        {
            perpendicular = QVector3D(-info.direction.y(), info.direction.x(), 0.0f);
        }

        if (perpendicular.lengthSquared() <= 1.0e-6f)
        {
            perpendicular = buildPerpendicularDirection(info.direction);
        }

        if (perpendicular.lengthSquared() > 1.0e-6f)
        {
            perpendicular.normalize();
        }

        if (perpendicular.lengthSquared() <= 1.0e-6f)
        {
            continue;
        }

        const QVector3D tip = info.startPoint;
        const QVector3D headBase = tip - info.direction * headLength;
        const QVector3D notch = tip - info.direction * (headLength * 0.72f);

        const QPoint tipScreen = worldToScreen(tip);
        const QPoint headBaseScreen = worldToScreen(headBase);
        const double projectedLength = std::hypot
        (
            static_cast<double>(tipScreen.x() - headBaseScreen.x()),
            static_cast<double>(tipScreen.y() - headBaseScreen.y())
        );

        if (projectedLength < 3.0)
        {
            continue;
        }

        QVector3D arrowColor;
        if (entity->m_isSelected)
        {
            arrowColor = QVector3D(1.0f, 0.78f, 0.30f);
        }
        else if (info.processOrder >= 0)
        {
            arrowColor = info.isReverse ? QVector3D(1.0f, 0.42f, 0.28f) : QVector3D(0.12f, 0.92f, 0.72f);
        }
        else
        {
            arrowColor = info.isReverse ? QVector3D(0.82f, 0.34f, 0.26f) : QVector3D(0.34f, 0.78f, 0.58f);
        }

        TransientPrimitive headPrimitive;
        headPrimitive.primitiveType = GL_TRIANGLES;
        headPrimitive.color = arrowColor;
        headPrimitive.vertices =
        {
            tip,
            headBase + perpendicular * headHalfWidth,
            notch,

            tip,
            notch,
            headBase - perpendicular * headHalfWidth
        };
        primitives.push_back(std::move(headPrimitive));
    }

    return primitives;
}

std::vector<TransientPrimitive> CadViewer::buildRotaryEndCutPrimitives() const
{
    if (!m_processVisualsVisible || !m_rotaryEndCutsVisible)
    {
        return {};
    }

    CadDocument* scene = m_sceneCoordinator.document();

    if (scene == nullptr)
    {
        return {};
    }

    std::vector<TransientPrimitive> primitives;
    const float pixelScale = std::max(pixelToWorldScale(), 1.0e-4f);
    const QVector3D screenOffset = m_camera.rightDirection().normalized() * pixelScale * 1.4f;

    for (const std::unique_ptr<CadItem>& entity : scene->m_entities)
    {
        if (entity == nullptr || m_processState == nullptr
            || entity->m_geometry.vertices.size() < 2)
        {
            continue;
        }

        const auto state = m_processState->stateOrDefault(entity->m_entityId);
        if (state.overrideData.boundaryRole == cadcam::planning::BoundaryRole::None
            || state.overrideData.boundaryPairId < 0) continue;

        const QVector3D color = state.overrideData.boundaryRole == cadcam::planning::BoundaryRole::Waste
            ? QVector3D(1.0f, 0.60f, 0.12f)
            : QVector3D(0.18f, 0.68f, 1.0f);

        for (int layer = -1; layer <= 1; ++layer)
        {
            TransientPrimitive path;
            path.primitiveType = GL_LINE_STRIP;
            path.color = color;
            path.opacity = layer == 0 ? 0.92f : 0.22f;
            path.vertices.reserve(entity->m_geometry.vertices.size());

            for (const QVector3D& point : entity->m_geometry.vertices)
            {
                path.vertices.push_back(point + screenOffset * static_cast<float>(layer));
            }

            primitives.push_back(std::move(path));
        }
    }

    return primitives;
}

std::vector<TransientPrimitive> CadViewer::buildSelectedEntityHandlePrimitives() const
{
    const CadItem* selectedItem = selectedEntity();

    if (selectedItem == nullptr)
    {
        return {};
    }

    const QVector<CadSelectionHandleInfo> handles = buildSelectionHandleInfo(selectedItem, xlineHandleWorldLength());

    if (handles.isEmpty())
    {
        return {};
    }

    const int hoveredHandleIndex = resolveHoveredHandleIndex(handles);

    TransientPrimitive basePointOuterPrimitive;
    basePointOuterPrimitive.primitiveType = GL_POINTS;
    basePointOuterPrimitive.color =
    {
        static_cast<float>(m_theme.selectedBasePointColor.redF()),
        static_cast<float>(m_theme.selectedBasePointColor.greenF()),
        static_cast<float>(m_theme.selectedBasePointColor.blueF())
    };
    basePointOuterPrimitive.pointSize = 13.0f;
    basePointOuterPrimitive.roundPoint = true;

    TransientPrimitive basePointInnerPrimitive;
    basePointInnerPrimitive.primitiveType = GL_POINTS;
    basePointInnerPrimitive.color =
    {
        static_cast<float>(m_theme.viewerBackgroundColor.redF()),
        static_cast<float>(m_theme.viewerBackgroundColor.greenF()),
        static_cast<float>(m_theme.viewerBackgroundColor.blueF())
    };
    basePointInnerPrimitive.pointSize = 6.2f;
    basePointInnerPrimitive.roundPoint = true;

    TransientPrimitive controlPointOuterPrimitive;
    controlPointOuterPrimitive.primitiveType = GL_POINTS;
    controlPointOuterPrimitive.color =
    {
        static_cast<float>(m_theme.selectedControlPointColor.redF()),
        static_cast<float>(m_theme.selectedControlPointColor.greenF()),
        static_cast<float>(m_theme.selectedControlPointColor.blueF())
    };
    controlPointOuterPrimitive.pointSize = 10.0f;
    controlPointOuterPrimitive.roundPoint = true;

    TransientPrimitive controlPointInnerPrimitive;
    controlPointInnerPrimitive.primitiveType = GL_POINTS;
    controlPointInnerPrimitive.color =
    {
        static_cast<float>(m_theme.viewerBackgroundColor.redF()),
        static_cast<float>(m_theme.viewerBackgroundColor.greenF()),
        static_cast<float>(m_theme.viewerBackgroundColor.blueF())
    };
    controlPointInnerPrimitive.pointSize = 4.4f;
    controlPointInnerPrimitive.roundPoint = true;

    TransientPrimitive trianglePrimitive;
    trianglePrimitive.primitiveType = GL_TRIANGLES;
    trianglePrimitive.color = controlPointOuterPrimitive.color;

    TransientPrimitive hoveredPointOuterPrimitive;
    hoveredPointOuterPrimitive.primitiveType = GL_POINTS;
    hoveredPointOuterPrimitive.color =
    {
        static_cast<float>(m_theme.accentColor.redF()),
        static_cast<float>(m_theme.accentColor.greenF()),
        static_cast<float>(m_theme.accentColor.blueF())
    };
    hoveredPointOuterPrimitive.pointSize = 17.0f;
    hoveredPointOuterPrimitive.roundPoint = true;

    TransientPrimitive hoveredPointInnerPrimitive;
    hoveredPointInnerPrimitive.primitiveType = GL_POINTS;
    hoveredPointInnerPrimitive.color = { 1.0f, 1.0f, 1.0f };
    hoveredPointInnerPrimitive.pointSize = 9.0f;
    hoveredPointInnerPrimitive.roundPoint = true;

    TransientPrimitive hoveredTrianglePrimitive;
    hoveredTrianglePrimitive.primitiveType = GL_TRIANGLES;
    hoveredTrianglePrimitive.color = hoveredPointOuterPrimitive.color;

    const float pixelScale = std::max(pixelToWorldScale(), 1.0e-4f);
    const float triangleSize = pixelScale * 6.5f;
    const float hoveredTriangleSize = pixelScale * 8.4f;

    for (int handleIndex = 0; handleIndex < handles.size(); ++handleIndex)
    {
        const CadSelectionHandleInfo& handle = handles.at(handleIndex);

        if (handle.shape == CadSelectionHandleShape::Triangle)
        {
            const QVector<QVector3D> triangleVertices = buildTriangleVertices(handle.position, handle.direction, triangleSize);
            trianglePrimitive.vertices += triangleVertices;

            if (handleIndex == hoveredHandleIndex)
            {
                const QVector<QVector3D> hoveredTriangleVertices = buildTriangleVertices
                (
                    handle.position,
                    handle.direction,
                    hoveredTriangleSize
                );
                hoveredTrianglePrimitive.vertices += hoveredTriangleVertices;
            }
        }
        else if (handle.isBasePoint)
        {
            basePointOuterPrimitive.vertices.push_back(handle.position);
            basePointInnerPrimitive.vertices.push_back(handle.position);
        }
        else
        {
            controlPointOuterPrimitive.vertices.push_back(handle.position);
            controlPointInnerPrimitive.vertices.push_back(handle.position);
        }

        if (handleIndex == hoveredHandleIndex)
        {
            hoveredPointOuterPrimitive.vertices.push_back(handle.position);
            hoveredPointInnerPrimitive.vertices.push_back(handle.position);
        }
    }

    std::vector<TransientPrimitive> primitives;

    if (!hoveredPointOuterPrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(hoveredPointOuterPrimitive));
    }

    if (!controlPointOuterPrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(controlPointOuterPrimitive));
    }

    if (!controlPointInnerPrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(controlPointInnerPrimitive));
    }

    if (!hoveredPointInnerPrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(hoveredPointInnerPrimitive));
    }

    if (!trianglePrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(trianglePrimitive));
    }

    if (!hoveredTrianglePrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(hoveredTrianglePrimitive));
    }

    if (!basePointOuterPrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(basePointOuterPrimitive));
    }

    if (!basePointInnerPrimitive.vertices.isEmpty())
    {
        primitives.push_back(std::move(basePointInnerPrimitive));
    }

    return primitives;
}
