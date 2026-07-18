#include "platform/pch.h"

#include "application/machining/RotaryTubeGeometryAnalyzer.h"

#include "cad/items/CadItem.h"
#include "application/geometry/GeometrySnapshot.h"
#include "application/topology/GeometrySnapshotTopologyAdapter.h"
#include "compatibility/legacy/LegacyCadItemTopologyAdapter.h"
#include "infrastructure/dxf/DxfGeometryAdapter.h"

#include <QHash>

namespace
{
    using cadcam::geometry::EntityId;
    using cadcam::machining::InternalPathClassification;
    using cadcam::machining::TubeCornerGeometry;
    using cadcam::machining::TubeCutBoundaryClassifier;
    using cadcam::machining::TubeSectionGeometry;
    using cadcam::machining::TubeSectionAnalyzer;
    using cadcam::machining::TubeSectionModel;
    using cadcam::machining::TubeSectionPolicy;
    using cadcam::topology::PathTopology;
    using cadcam::topology::PathTopologyBuilder;
    using cadcam::topology::PathTopologyTolerance;
    using cadcam::topology::TopologyInput;

    struct PreparedTopology
    {
        bool valid = false;
        TopologyInput input;
        PathTopology topology;
        QVector<Diagnostic> diagnostics;
    };

    QString firstDiagnosticMessage(const QVector<Diagnostic>& diagnostics)
    {
        for (const Diagnostic& diagnostic : diagnostics)
        {
            if (isErrorSeverity(diagnostic.severity) && !diagnostic.userMessage.isEmpty())
            {
                return diagnostic.userMessage;
            }
        }

        for (const Diagnostic& diagnostic : diagnostics)
        {
            if (!diagnostic.userMessage.isEmpty())
            {
                return diagnostic.userMessage;
            }
        }

        return {};
    }

    TubeSectionPolicy buildPolicy(double connectionTolerance)
    {
        TubeSectionPolicy policy;
        policy.connectionTolerance = std::max(0.001, connectionTolerance);
        policy.numericalEpsilon = PathTopologyTolerance{}.numericalJoinEpsilon;
        policy.maximumPlaneDeviation = 0.1;
        policy.boundaryDistanceTolerance = std::max
            (0.001, policy.connectionTolerance * 0.1);
        policy.interiorDistanceTolerance = std::max
            (0.0001, policy.connectionTolerance * 0.01);
        return policy;
    }

    PreparedTopology prepareTopology
    (
        const QVector<CadItem*>& sceneItems,
        double connectionTolerance,
        std::uint64_t contentRevision,
        const OperationContext& context
    )
    {
        PreparedTopology prepared;
        const PathTopologyTolerance tolerance =
            PathTopologyTolerance::fromConnectionTolerance(connectionTolerance);
        LegacyCadItemTopologyAdapter adapter;
        OperationResult<TopologyInput> converted = adapter.convert
            (sceneItems, tolerance, context);
        prepared.diagnostics += converted.diagnostics;

        if (!converted.succeeded() || !converted.value.has_value())
        {
            return prepared;
        }
        converted.value->contentRevision = contentRevision;

        TaskContext taskContext;
        taskContext.operationContext = context;
        const OperationResult<PathTopology> built = PathTopologyBuilder{}.build
            (*converted.value, tolerance, taskContext);
        prepared.diagnostics += built.diagnostics;

        if (!built.succeeded() || !built.value.has_value())
        {
            return prepared;
        }

        prepared.valid = true;
        prepared.input = *converted.value;
        prepared.topology = *built.value;
        return prepared;
    }

    PreparedTopology prepareCompiledTopology
    (
        const QVector<CadItem*>& sceneItems,
        double connectionTolerance,
        std::uint64_t contentRevision,
        const OperationContext& context
    )
    {
        PreparedTopology prepared;
        GeometrySnapshot snapshot;
        snapshot.contentRevision = std::max<std::uint64_t>(1U, contentRevision);
        snapshot.entries.reserve(static_cast<std::size_t>(sceneItems.size()));
        cadcam::geometry::GeometryCompiler compiler;
        cadcam::geometry::SamplingPolicy policy;
        cadcam::geometry::PathCompileOptions options;

        for (qsizetype index = 0; index < sceneItems.size(); ++index)
        {
            CadItem* item = sceneItems[index];
            if (item == nullptr || item->m_nativeEntity == nullptr || item->m_entityId == 0U)
            {
                continue;
            }

            GeometrySnapshotEntry entry;
            entry.sourceIndex = static_cast<std::size_t>(index);
            entry.attributes.entityId = item->m_entityId;
            entry.attributes.originalDxfType = static_cast<int>(item->m_type);
            const OperationResult<cadcam::geometry::SourceEntity> source =
                DxfGeometryAdapter::convert(item->m_entityId, *item->m_nativeEntity, context);
            entry.diagnostics += source.diagnostics;
            if (source.value.has_value())
            {
                entry.sourceKind = source.value->kind;
                OperationResult<cadcam::geometry::Path3D> path = compiler.compile
                    (*source.value, policy, options, context);
                entry.diagnostics += path.diagnostics;
                entry.status = source.status == OperationStatus::PartialSuccess
                    || path.status == OperationStatus::PartialSuccess
                    ? OperationStatus::PartialSuccess : path.status;
                if (path.value.has_value()) entry.path = std::move(*path.value);
            }
            else
            {
                entry.status = source.status;
            }
            if (entry.path.has_value()) prepared.diagnostics += entry.diagnostics;
            snapshot.entries.push_back(std::move(entry));
        }

        const PathTopologyTolerance tolerance =
            PathTopologyTolerance::fromConnectionTolerance(connectionTolerance);
        const OperationResult<TopologyInput> converted =
            GeometrySnapshotTopologyAdapter{}.convert(snapshot, {}, tolerance, context);
        prepared.diagnostics += converted.diagnostics;
        if (!converted.succeeded() || !converted.value.has_value()) return prepared;

        TaskContext taskContext;
        taskContext.operationContext = context;
        const OperationResult<PathTopology> built = PathTopologyBuilder{}.build
            (*converted.value, tolerance, taskContext);
        prepared.diagnostics += built.diagnostics;
        if (!built.succeeded() || !built.value.has_value()) return prepared;

        prepared.valid = true;
        prepared.input = *converted.value;
        prepared.topology = *built.value;
        return prepared;
    }

    QHash<EntityId, CadItem*> itemMap(const QVector<CadItem*>& sceneItems)
    {
        QHash<EntityId, CadItem*> result;

        for (CadItem* item : sceneItems)
        {
            if (item != nullptr && item->m_entityId != 0U)
            {
                result.insert(item->m_entityId, item);
            }
        }

        return result;
    }

    RotaryTubeSectionModel toLegacyModel
    (
        const TubeSectionModel& core,
        const QVector<CadItem*>& sceneItems,
        const QVector<Diagnostic>& diagnostics
    )
    {
        RotaryTubeSectionModel model;
        model.valid = true;
        model.coreModel = core;
        model.yLength = core.geometry.yLength;
        model.zWidth = core.geometry.zWidth;
        model.cornerRadius = core.cornerRadius;
        model.roundedCornerCount = core.roundedCornerCount;
        model.cornerConfidence = core.cornerConfidence;
        model.centerX = core.centerX;
        model.centerValid = true;
        model.centerY = core.geometry.centerY;
        model.centerZ = core.geometry.centerZ;
        model.cornerRadii.reserve(static_cast<qsizetype>(core.cornerRadii.size()));

        for (const double radius : core.cornerRadii)
        {
            model.cornerRadii.push_back(radius);
        }

        model.sectionBoundary.reserve(static_cast<qsizetype>(core.geometry.boundary.size()));

        for (const cadcam::geometry::Vector2d& point : core.geometry.boundary)
        {
            model.sectionBoundary.push_back
                (QVector2D(static_cast<float>(point.x), static_cast<float>(point.y)));
        }

        const QHash<EntityId, CadItem*> byId = itemMap(sceneItems);

        for (const EntityId entityId : core.outerBoundaryEntityIds)
        {
            const auto item = byId.constFind(entityId);

            if (item != byId.cend())
            {
                model.outerBoundaryItems.push_back(item.value());
            }
        }

        for (const Diagnostic& diagnostic : diagnostics)
        {
            if (diagnostic.context.contains(QStringLiteral("candidateCount")))
            {
                model.inspectedCandidateCount = diagnostic.context
                    .value(QStringLiteral("candidateCount")).toInt();
            }
            if (diagnostic.context.contains(QStringLiteral("validCandidateCount")))
            {
                model.validCandidateCount = diagnostic.context
                    .value(QStringLiteral("validCandidateCount")).toInt();
            }
            if (diagnostic.context.contains(QStringLiteral("roundedCandidateCount")))
            {
                model.roundedCandidateCount = diagnostic.context
                    .value(QStringLiteral("roundedCandidateCount")).toInt();
            }
        }

        return model;
    }

    std::vector<EntityId> selectedIds(const QVector<CadItem*>& selectedItems)
    {
        std::vector<EntityId> ids;
        ids.reserve(static_cast<std::size_t>(selectedItems.size()));

        for (const CadItem* item : selectedItems)
        {
            if (item != nullptr && item->m_entityId != 0U)
            {
                ids.push_back(item->m_entityId);
            }
        }

        return ids;
    }
}

RotaryTubeSectionModel RotaryTubeGeometryAnalyzer::buildSectionModel
(
    const QVector<CadItem*>& selectedItems,
    const QVector<CadItem*>& sceneItems,
    double connectionTolerance,
    std::uint64_t contentRevision
)
{
    RotaryTubeSectionModel model;
    const OperationContext context = createOperationContext
        (QStringLiteral("BuildTubeSectionFromSelection"));

    if (selectedItems.isEmpty())
    {
        model.errorMessage = QStringLiteral("请先选中方管垂直截面中的一个或部分图元。");
        return model;
    }

    const PreparedTopology prepared = prepareTopology
        (sceneItems, connectionTolerance, contentRevision, context);

    if (!prepared.valid)
    {
        model.errorMessage = firstDiagnosticMessage(prepared.diagnostics);
        return model;
    }

    const OperationResult<TubeSectionModel> analyzed = TubeSectionAnalyzer::buildFromSelection
    (
        prepared.input,
        prepared.topology,
        selectedIds(selectedItems),
        buildPolicy(connectionTolerance),
        context
    );
    QVector<Diagnostic> diagnostics = prepared.diagnostics;
    diagnostics += analyzed.diagnostics;

    if (!analyzed.succeeded() || !analyzed.value.has_value())
    {
        model.errorMessage = firstDiagnosticMessage(diagnostics);
        return model;
    }

    return toLegacyModel(*analyzed.value, sceneItems, diagnostics);
}

RotaryTubeSectionModel RotaryTubeGeometryAnalyzer::findBestSectionModel
(
    const QVector<CadItem*>& sceneItems,
    double connectionTolerance,
    std::uint64_t contentRevision
)
{
    RotaryTubeSectionModel model;
    const OperationContext context = createOperationContext(QStringLiteral("FindBestTubeSection"));
    const PreparedTopology prepared = prepareTopology
        (sceneItems, connectionTolerance, contentRevision, context);

    if (!prepared.valid)
    {
        model.errorMessage = firstDiagnosticMessage(prepared.diagnostics);
        return model;
    }

    const OperationResult<TubeSectionModel> analyzed = TubeSectionAnalyzer::findBest
    (
        prepared.input,
        prepared.topology,
        buildPolicy(connectionTolerance),
        context
    );
    QVector<Diagnostic> diagnostics = prepared.diagnostics;
    diagnostics += analyzed.diagnostics;

    if (!analyzed.succeeded() || !analyzed.value.has_value())
    {
        model.errorMessage = firstDiagnosticMessage(diagnostics);
        return model;
    }

    return toLegacyModel(*analyzed.value, sceneItems, diagnostics);
}

RotaryTubeSectionModel RotaryTubeGeometryAnalyzer::buildManualSectionModel
(
    double yLength,
    double zWidth,
    double cornerRadius,
    double centerX,
    double centerY,
    double centerZ,
    std::uint64_t contentRevision
)
{
    RotaryTubeSectionModel model;
    constexpr double epsilon = 1.0e-5;

    if (!std::isfinite(yLength) || !std::isfinite(zWidth)
        || !std::isfinite(cornerRadius) || !std::isfinite(centerX)
        || !std::isfinite(centerY) || !std::isfinite(centerZ)
        || yLength <= epsilon || zWidth <= epsilon || cornerRadius < 0.0)
    {
        model.errorMessage = QStringLiteral("Y 长和 Z 宽必须大于 0，圆角半径不能为负数。");
        return model;
    }

    const double maximumRadius = std::min(yLength, zWidth) * 0.5;
    if (cornerRadius > maximumRadius + epsilon)
    {
        model.errorMessage = QStringLiteral("圆角半径不能超过 Y 长和 Z 宽较小值的一半。");
        return model;
    }

    cornerRadius = std::min(cornerRadius, maximumRadius);
    const double halfY = yLength * 0.5;
    const double halfZ = zWidth * 0.5;
    const double minimumY = centerY - halfY;
    const double maximumY = centerY + halfY;
    const double minimumZ = centerZ - halfZ;
    const double maximumZ = centerZ + halfZ;
    TubeSectionGeometry geometry;
    geometry.centerY = centerY;
    geometry.centerZ = centerZ;
    geometry.yLength = yLength;
    geometry.zWidth = zWidth;

    if (cornerRadius <= epsilon)
    {
        geometry.boundary =
        {
            { minimumY, maximumZ },
            { maximumY, maximumZ },
            { maximumY, minimumZ },
            { minimumY, minimumZ }
        };
    }
    else
    {
        constexpr int arcSegments = 16;
        constexpr double halfPi = 1.57079632679489661923;
        constexpr double pi = 3.14159265358979323846;
        auto appendArc = [&geometry, cornerRadius]
        (
            double arcCenterY,
            double arcCenterZ,
            double startAngle,
            double endAngle
        )
        {
            for (int segment = 1; segment <= arcSegments; ++segment)
            {
                const double ratio = static_cast<double>(segment) / arcSegments;
                const double angle = startAngle + (endAngle - startAngle) * ratio;
                geometry.boundary.push_back
                ({
                    arcCenterY + cornerRadius * std::cos(angle),
                    arcCenterZ + cornerRadius * std::sin(angle)
                });
            }
        };

        geometry.boundary.push_back({ minimumY + cornerRadius, maximumZ });
        geometry.boundary.push_back({ maximumY - cornerRadius, maximumZ });
        appendArc(maximumY - cornerRadius, maximumZ - cornerRadius, halfPi, 0.0);
        geometry.boundary.push_back({ maximumY, minimumZ + cornerRadius });
        appendArc(maximumY - cornerRadius, minimumZ + cornerRadius, 0.0, -halfPi);
        geometry.boundary.push_back({ minimumY + cornerRadius, minimumZ });
        appendArc(minimumY + cornerRadius, minimumZ + cornerRadius, -halfPi, -pi);
        geometry.boundary.push_back({ minimumY, maximumZ - cornerRadius });
        appendArc(minimumY + cornerRadius, maximumZ - cornerRadius, pi, halfPi);
    }

    const OperationContext context = createOperationContext
        (QStringLiteral("BuildManualTubeSection"));
    const OperationResult<TubeSectionGeometry> prepared =
        TubeCutBoundaryClassifier::prepareSection(geometry, context, epsilon);
    if (!prepared.succeeded() || !prepared.value.has_value())
    {
        model.errorMessage = QStringLiteral("手动方管截面参数无法形成有效边界。");
        return model;
    }

    TubeSectionModel core;
    core.contentRevision = contentRevision;
    core.geometry = *prepared.value;
    core.centerX = centerX;
    core.roundedCornerCount = cornerRadius > epsilon ? 4 : 0;
    core.cornerRadius = cornerRadius;
    core.cornerConfidence = 1.0;
    if (core.roundedCornerCount == 4)
    {
        core.cornerRadii.assign(4, cornerRadius);
        for (const int yDirection : { -1, 1 })
        {
            for (const int zDirection : { -1, 1 })
            {
                core.corners.push_back
                (TubeCornerGeometry
                {
                    {
                        centerY + yDirection * (halfY - cornerRadius),
                        centerZ + zDirection * (halfZ - cornerRadius)
                    },
                    cornerRadius,
                    yDirection,
                    zDirection
                });
            }
        }
    }
    core.orderedBoundary3D.reserve(core.geometry.boundary.size());
    for (const cadcam::geometry::Vector2d& point : core.geometry.boundary)
    {
        core.orderedBoundary3D.push_back({ centerX, point.x, point.y });
    }

    model = toLegacyModel(core, {}, {});
    model.manuallyConfigured = true;
    return model;
}

RotaryInternalPathResult RotaryTubeGeometryAnalyzer::findInternalPaths
(
    const RotaryTubeSectionModel& model,
    const QVector<CadItem*>& sceneItems,
    double connectionTolerance
)
{
    RotaryInternalPathResult result;

    const OperationContext context = createOperationContext
        (QStringLiteral("ClassifyTubeInternalPaths"));
    const std::uint64_t contentRevision = model.coreModel.has_value()
        ? model.coreModel->contentRevision : 1U;
    const PreparedTopology prepared = prepareCompiledTopology
        (sceneItems, connectionTolerance, contentRevision, context);
    result.diagnostics = prepared.diagnostics;

    if (!prepared.valid)
    {
        return result;
    }

    const OperationResult<InternalPathClassification> classified = model.coreModel.has_value()
        ? TubeSectionAnalyzer::classifyInternalPaths
        (
            prepared.input,
            prepared.topology,
            *model.coreModel,
            buildPolicy(connectionTolerance),
            context
        )
        : TubeSectionAnalyzer::classifyTopologicalInteriorPaths
        (
            prepared.input,
            prepared.topology,
            context
        );
    result.diagnostics += classified.diagnostics;

    if (!classified.succeeded() || !classified.value.has_value())
    {
        return result;
    }
    result.skippedComponentCount = classified.value->skippedComponentCount;

    const QHash<EntityId, CadItem*> byId = itemMap(sceneItems);

    for (const EntityId entityId : classified.value->physicalInteriorEntityIds)
    {
        const auto item = byId.constFind(entityId);
        if (item != byId.cend()) result.physicalInteriorItems.push_back(item.value());
    }

    for (const EntityId entityId : classified.value->topologicalInteriorEntityIds)
    {
        const auto item = byId.constFind(entityId);
        if (item != byId.cend()) result.topologicalInteriorItems.push_back(item.value());
    }

    return result;
}
