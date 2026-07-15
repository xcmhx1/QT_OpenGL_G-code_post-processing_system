#include "pch.h"

#include "RotaryTubeGeometryAnalyzer.h"

#include "CadItem.h"
#include "compatibility/legacy/LegacyCadItemTopologyAdapter.h"

#include <QHash>

namespace
{
    using cadcam::geometry::EntityId;
    using cadcam::machining::InternalPathClassification;
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

RotaryInternalPathResult RotaryTubeGeometryAnalyzer::findInternalPaths
(
    const RotaryTubeSectionModel& model,
    const QVector<CadItem*>& sceneItems,
    double connectionTolerance
)
{
    RotaryInternalPathResult result;

    if (!model.coreModel.has_value())
    {
        return result;
    }

    const OperationContext context = createOperationContext
        (QStringLiteral("ClassifyTubeInternalPaths"));
    const PreparedTopology prepared = prepareTopology
        (sceneItems, connectionTolerance, model.coreModel->contentRevision, context);

    if (!prepared.valid)
    {
        return result;
    }

    const OperationResult<InternalPathClassification> classified =
        TubeSectionAnalyzer::classifyInternalPaths
    (
        prepared.input,
        prepared.topology,
        *model.coreModel,
        buildPolicy(connectionTolerance),
        context
    );

    if (!classified.succeeded() || !classified.value.has_value())
    {
        return result;
    }

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
