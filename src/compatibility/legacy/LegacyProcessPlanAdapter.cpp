#include "compatibility/legacy/LegacyProcessPlanAdapter.h"

#include "CadDocument.h"
#include "CadItem.h"
#include "application/tasks/TaskContext.h"
#include "compatibility/legacy/LegacyCadItemTopologyAdapter.h"

#include <QThread>
#include <QVector>

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace
{
    using cadcam::geometry::EntityId;
    using cadcam::geometry::Path3D;
    using cadcam::geometry::PathVertex3D;
    using cadcam::geometry::SourceGeometryKind;
    using cadcam::planning::BoundaryRole;
    using cadcam::planning::PlanningEntity;
    using cadcam::planning::ProcessPlan;
    using cadcam::planning::ProcessPlanningInput;

    SourceGeometryKind sourceKind(DRW::ETYPE type)
    {
        switch (type)
        {
        case DRW::LINE: return SourceGeometryKind::Line;
        case DRW::ARC: return SourceGeometryKind::Arc;
        case DRW::CIRCLE: return SourceGeometryKind::Circle;
        case DRW::ELLIPSE: return SourceGeometryKind::Ellipse;
        case DRW::POLYLINE:
        case DRW::LWPOLYLINE: return SourceGeometryKind::Polyline;
        case DRW::SPLINE: return SourceGeometryKind::Spline;
        case DRW::POINT: return SourceGeometryKind::Point;
        default: return SourceGeometryKind::Unknown;
        }
    }

    BoundaryRole boundaryRole(RotaryEndCutRole role)
    {
        switch (role)
        {
        case RotaryEndCutRole::Break: return BoundaryRole::Break;
        case RotaryEndCutRole::Waste: return BoundaryRole::Waste;
        default: return BoundaryRole::None;
        }
    }

    bool supportsProcessPath(SourceGeometryKind kind)
    {
        return kind == SourceGeometryKind::Line
            || kind == SourceGeometryKind::Arc
            || kind == SourceGeometryKind::Circle
            || kind == SourceGeometryKind::Ellipse
            || kind == SourceGeometryKind::Polyline
            || kind == SourceGeometryKind::Spline;
    }

    bool hasUsableTopologyPath(CadItem& item, double minimumEdgeLength)
    {
        item.rebuildRawPathPoints3D();
        const std::vector<RawPathPoint3D>& points = item.rawPathPoints3D();
        if (points.size() < 2U) return false;
        if (!std::isfinite(points.front().x) || !std::isfinite(points.front().y)
            || !std::isfinite(points.front().z)) return false;

        std::size_t distinctPointCount = 1U;
        const RawPathPoint3D* previous = &points.front();
        for (std::size_t index = 1U; index < points.size(); ++index)
        {
            const RawPathPoint3D& current = points[index];
            if (!std::isfinite(current.x) || !std::isfinite(current.y)
                || !std::isfinite(current.z)) return false;
            const double dx = current.x - previous->x;
            const double dy = current.y - previous->y;
            const double dz = current.z - previous->z;
            if (std::sqrt(dx * dx + dy * dy + dz * dz) <= minimumEdgeLength) continue;
            ++distinctPointCount;
            previous = &current;
        }
        return distinctPointCount >= 2U;
    }

    Diagnostic adapterDiagnostic
    (
        const OperationContext& context,
        DiagnosticCode code,
        const QString& message,
        const QString& detail,
        std::uint64_t revision,
        EntityId entityId = 0,
        std::size_t sourceIndex = 0
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = code;
        diagnostic.severity = DiagnosticSeverity::Error;
        diagnostic.component = QStringLiteral("LegacyProcessPlanAdapter");
        diagnostic.operation = QStringLiteral("AdaptProcessPlan");
        diagnostic.stage = QStringLiteral("DocumentThread");
        diagnostic.userMessage = message;
        diagnostic.technicalDetail = detail;
        diagnostic.correlationId = context.correlationId;
        diagnostic.context.insert(QStringLiteral("contentRevision"), QVariant::fromValue<qulonglong>(revision));
        diagnostic.context.insert(QStringLiteral("entityId"), QVariant::fromValue<qulonglong>(entityId));
        diagnostic.context.insert(QStringLiteral("sourceIndex"), QVariant::fromValue<qulonglong>(sourceIndex));
        if (entityId != 0U) diagnostic.entityId = entityId;
        return diagnostic;
    }
}

OperationResult<cadcam::planning::ProcessPlanningInput> LegacyProcessPlanAdapter::capture
(
    CadDocument& document,
    const std::optional<cadcam::machining::TubeSectionModel>& tubeSection,
    double connectionTolerance,
    cadcam::topology::PathTopology& topologyStorage,
    const OperationContext& context
) const
{
    OperationResult<ProcessPlanningInput> result;
    if (QThread::currentThread() != document.thread() || document.contentRevision() == 0U)
    {
        result.status = OperationStatus::InvalidInput;
        result.addDiagnostic(adapterDiagnostic
        (
            context, DiagnosticCode::ProcessPlanningInputInvalid,
            QStringLiteral("加工计划输入必须在文档线程捕获。"),
            QStringLiteral("Wrong thread or zero document revision."), document.contentRevision()
        ));
        return result;
    }

    const cadcam::topology::PathTopologyTolerance topologyTolerance =
        cadcam::topology::PathTopologyTolerance::fromConnectionTolerance(connectionTolerance);
    QVector<CadItem*> topologyItems;
    std::unordered_map<EntityId, std::size_t> sourceIndices;
    topologyItems.reserve(static_cast<qsizetype>(document.m_entities.size()));
    for (std::size_t index = 0; index < document.m_entities.size(); ++index)
    {
        CadItem* item = document.m_entities[index].get();
        if (item == nullptr) continue;
        sourceIndices.emplace(item->m_entityId, index);
        const SourceGeometryKind kind = sourceKind(item->m_type);
        if (item->m_nativeEntity != nullptr && supportsProcessPath(kind)
            && hasUsableTopologyPath(*item, topologyTolerance.minimumEdgeLength))
        {
            topologyItems.push_back(item);
        }
    }

    cadcam::topology::TopologyInput topologyInput;
    topologyInput.contentRevision = document.contentRevision();
    QVector<Diagnostic> topologyDiagnostics;
    if (!topologyItems.isEmpty())
    {
        LegacyCadItemTopologyAdapter topologyAdapter;
        auto adapted = topologyAdapter.convert(topologyItems, topologyTolerance, context);
        if (!adapted.succeeded() || !adapted.value.has_value())
        {
            result.status = OperationStatus::Failed;
            result.mergeDiagnostics(adapted.diagnostics);
            result.addDiagnostic(adapterDiagnostic
            (
                context, DiagnosticCode::ProcessPlanningInputInvalid,
                QStringLiteral("无法捕获加工路径拓扑。"),
                QStringLiteral("LegacyCadItemTopologyAdapter failed."), document.contentRevision()
            ));
            return result;
        }
        topologyInput = std::move(*adapted.value);
        topologyInput.contentRevision = document.contentRevision();
        topologyDiagnostics = adapted.diagnostics;
        for (cadcam::topology::TopologyPathRecord& record : topologyInput.records)
        {
            const auto found = sourceIndices.find(record.entityId);
            if (found != sourceIndices.end()) record.sourceIndex = found->second;
        }

        TaskContext taskContext;
        taskContext.operationContext = context;
        cadcam::topology::PathTopologyBuilder topologyBuilder;
        auto topologyResult = topologyBuilder.build(topologyInput, topologyTolerance, taskContext);
        if (!topologyResult.succeeded() || !topologyResult.value.has_value())
        {
            result.status = OperationStatus::Failed;
            result.mergeDiagnostics(topologyDiagnostics);
            result.mergeDiagnostics(topologyResult.diagnostics);
            return result;
        }
        topologyStorage = std::move(*topologyResult.value);
    }

    std::unordered_map<EntityId, const cadcam::topology::TopologyPathRecord*> records;
    for (const cadcam::topology::TopologyPathRecord& record : topologyInput.records)
        records.emplace(record.entityId, &record);

    ProcessPlanningInput input;
    input.contentRevision = document.contentRevision();
    input.topologyInput = std::move(topologyInput);
    input.topology = &topologyStorage;
    input.tubeSection = tubeSection;
    input.entities.reserve(document.m_entities.size());

    for (std::size_t index = 0; index < document.m_entities.size(); ++index)
    {
        CadItem* item = document.m_entities[index].get();
        if (item == nullptr) continue;
        PlanningEntity entity;
        entity.entityId = item->m_entityId;
        entity.sourceIndex = index;
        entity.sourceKind = sourceKind(item->m_type);
        entity.visible = true;
        entity.processEnabled = item->m_nativeEntity != nullptr;
        entity.excludedAsInternalGeometry = item->m_excludedAsInternalGeometry;
        entity.boundaryRole = boundaryRole(item->m_rotaryEndCutRole);
        entity.boundaryPairId = item->m_rotaryEndCutPairId;
        entity.currentReverse = item->m_isReverse;
        if (item->m_hasCustomProcessStart)
            entity.currentStartParameter = item->m_processStartParameter;

        const auto found = records.find(item->m_entityId);
        if (found != records.end())
        {
            const cadcam::topology::TopologyPathRecord& record = *found->second;
            entity.path.sourceEntityId = entity.entityId;
            entity.path.sourceKind = entity.sourceKind;
            entity.path.closed = record.semanticallyClosed;
            entity.path.vertices.reserve(record.points.size());
            for (std::size_t pointIndex = 0; pointIndex < record.points.size(); ++pointIndex)
            {
                entity.path.vertices.push_back
                ({ record.points[pointIndex], static_cast<double>(pointIndex) });
            }
            if (entity.path.closed && entity.path.vertices.size() > 1U)
            {
                const auto& first = entity.path.vertices.front().position;
                const auto& last = entity.path.vertices.back().position;
                const double gap = std::sqrt
                (
                    (first.x - last.x) * (first.x - last.x)
                    + (first.y - last.y) * (first.y - last.y)
                    + (first.z - last.z) * (first.z - last.z)
                );
                if (gap <= topologyTolerance.numericalJoinEpsilon)
                    entity.path.vertices.pop_back();
            }
        }
        input.entities.push_back(std::move(entity));
    }

    result.status = OperationStatus::Success;
    result.value = std::move(input);
    result.mergeDiagnostics(topologyDiagnostics);
    return result;
}

OperationReport LegacyProcessPlanAdapter::apply
(
    CadDocument& document,
    const ProcessPlan& plan,
    const OperationContext& context
) const
{
    OperationReport result;
    if (QThread::currentThread() != document.thread() || document.contentRevision() != plan.contentRevision)
    {
        result.status = OperationStatus::Conflict;
        result.addDiagnostic(adapterDiagnostic
        (
            context, DiagnosticCode::ProcessPlanApplyConflict,
            QStringLiteral("文档已经变化，当前加工计划不能应用。"),
            QStringLiteral("Document thread or content revision mismatch."), document.contentRevision()
        ));
        return result;
    }

    std::unordered_map<EntityId, CadItem*> items;
    for (const std::unique_ptr<CadItem>& item : document.m_entities)
    {
        if (item == nullptr || item->m_entityId == 0U
            || !items.emplace(item->m_entityId, item.get()).second)
        {
            result.status = OperationStatus::Conflict;
            result.addDiagnostic(adapterDiagnostic
            (
                context, DiagnosticCode::ProcessPlanApplyConflict,
                QStringLiteral("文档中的图元编号无效，计划未应用。"),
                QStringLiteral("Document contains null, zero, or duplicate EntityId."),
                document.contentRevision(), item != nullptr ? item->m_entityId : 0U
            ));
            return result;
        }
    }

    if (!plan.precedenceConstraints.empty())
    {
        std::unordered_map<int, int> indegree;
        std::unordered_map<int, std::vector<int>> successors;
        for (const cadcam::planning::ProcessGroup& group : plan.groups)
        {
            if (group.groupId < 0 || !indegree.emplace(group.groupId, 0).second)
            {
                result.status = OperationStatus::Conflict;
                result.addDiagnostic(adapterDiagnostic
                (
                    context, DiagnosticCode::ProcessPlanApplyConflict,
                    QStringLiteral("加工计划中的加工组编号无效，计划未应用。"),
                    QStringLiteral("ProcessGroup ID is negative or duplicated."),
                    document.contentRevision()
                ));
                return result;
            }
        }
        for (const cadcam::planning::ProcessPrecedence& precedence : plan.precedenceConstraints)
        {
            if (indegree.find(precedence.predecessorGroupId) == indegree.end()
                || indegree.find(precedence.successorGroupId) == indegree.end())
            {
                result.status = OperationStatus::Conflict;
                result.addDiagnostic(adapterDiagnostic
                (
                    context, DiagnosticCode::ProcessPlanApplyConflict,
                    QStringLiteral("加工计划引用了缺失的加工组，计划未应用。"),
                    QStringLiteral("ProcessPrecedence references an unknown group."),
                    document.contentRevision()
                ));
                return result;
            }
            ++indegree[precedence.successorGroupId];
            successors[precedence.predecessorGroupId].push_back(precedence.successorGroupId);
        }
        std::queue<int> eligible;
        for (const auto& [groupId, count] : indegree)
            if (count == 0) eligible.push(groupId);
        std::size_t visitedGroupCount = 0U;
        while (!eligible.empty())
        {
            const int groupId = eligible.front();
            eligible.pop();
            ++visitedGroupCount;
            for (const int successor : successors[groupId])
                if (--indegree[successor] == 0) eligible.push(successor);
        }
        if (visitedGroupCount != indegree.size())
        {
            result.status = OperationStatus::Conflict;
            result.addDiagnostic(adapterDiagnostic
            (
                context, DiagnosticCode::ProcessPlanningPrecedenceCycle,
                QStringLiteral("中断切面前置约束形成循环，计划未应用。"),
                QStringLiteral("ProcessPlan precedence graph contains a cycle."),
                document.contentRevision()
            ));
            return result;
        }
    }

    std::unordered_set<EntityId> referenced;
    std::unordered_set<int> processOrders;
    for (const cadcam::planning::ProcessAssignment& assignment : plan.assignments)
    {
        if (assignment.processOrder < 0 || items.find(assignment.entityId) == items.end()
            || !referenced.insert(assignment.entityId).second
            || !processOrders.insert(assignment.processOrder).second)
        {
            result.status = OperationStatus::Conflict;
            result.addDiagnostic(adapterDiagnostic
            (
                context, DiagnosticCode::ProcessPlanEntityMissing,
                QStringLiteral("加工计划引用了缺失或重复的图元，计划未应用。"),
                QStringLiteral("Assignment validation failed."), document.contentRevision(), assignment.entityId
            ));
            return result;
        }
    }
    for (const cadcam::planning::ProcessExclusion& exclusion : plan.exclusions)
    {
        if (items.find(exclusion.entityId) == items.end() || !referenced.insert(exclusion.entityId).second)
        {
            result.status = OperationStatus::Conflict;
            result.addDiagnostic(adapterDiagnostic
            (
                context, DiagnosticCode::ProcessPlanEntityMissing,
                QStringLiteral("加工计划排除项引用了缺失或重复的图元，计划未应用。"),
                QStringLiteral("Exclusion validation failed."), document.contentRevision(), exclusion.entityId
            ));
            return result;
        }
    }
    if (referenced.size() != items.size()
        || (!processOrders.empty()
            && *std::max_element(processOrders.cbegin(), processOrders.cend())
                != static_cast<int>(processOrders.size()) - 1))
    {
        result.status = OperationStatus::Conflict;
        result.addDiagnostic(adapterDiagnostic
        (
            context, DiagnosticCode::ProcessPlanApplyConflict,
            QStringLiteral("加工计划未完整覆盖当前文档，计划未应用。"),
            QStringLiteral("Assignments/exclusions do not cover the document or process orders are not contiguous."),
            document.contentRevision()
        ));
        return result;
    }

    for (auto& [entityId, item] : items)
    {
        (void)entityId;
        item->m_processOrder = -1;
        item->m_processContinuousGroupId = -1;
        item->m_excludedFromProcessing = false;
    }
    for (const cadcam::planning::ProcessExclusion& exclusion : plan.exclusions)
    {
        CadItem* item = items.at(exclusion.entityId);
        item->m_excludedFromProcessing = true;
    }
    for (const cadcam::planning::ProcessAssignment& assignment : plan.assignments)
    {
        CadItem* item = items.at(assignment.entityId);
        item->m_processOrder = assignment.processOrder;
        item->m_processContinuousGroupId = assignment.continuousGroupId;
        item->m_isReverse = assignment.reverse;
        item->m_hasCustomProcessStart = assignment.startParameter.has_value();
        item->m_processStartParameter = assignment.startParameter.value_or(0.0);
        item->m_excludedFromProcessing = false;
    }
    document.notifySceneChanged();
    result.status = OperationStatus::Success;
    result.value = std::monostate{};
    return result;
}
