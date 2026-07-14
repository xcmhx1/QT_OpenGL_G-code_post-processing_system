#include "pch.h"

#include "RotaryPathTopology.h"

#include "CadItem.h"
#include "compatibility/legacy/LegacyCadItemTopologyAdapter.h"

#include <QMap>
#include <QStringList>

#include <algorithm>
#include <map>
#include <set>

namespace
{
    using cadcam::geometry::EntityId;
    using cadcam::geometry::Vector3d;
    using cadcam::topology::TopologyLoopResult;
    using cadcam::topology::TopologyPathRecord;

    QString entityTypeLabel(DRW::ETYPE type)
    {
        switch (type)
        {
        case DRW::LINE: return QStringLiteral("LINE");
        case DRW::ARC: return QStringLiteral("ARC");
        case DRW::CIRCLE: return QStringLiteral("CIRCLE");
        case DRW::ELLIPSE: return QStringLiteral("ELLIPSE");
        case DRW::LWPOLYLINE: return QStringLiteral("LWPOLYLINE");
        case DRW::POLYLINE: return QStringLiteral("POLYLINE");
        case DRW::SPLINE: return QStringLiteral("SPLINE");
        default: return QString::number(static_cast<int>(type));
        }
    }

    QVector3D toLegacyPoint(const Vector3d& point)
    {
        return QVector3D
        (
            static_cast<float>(point.x),
            static_cast<float>(point.y),
            static_cast<float>(point.z)
        );
    }

    QString compatibilityError(const OperationResult<TopologyLoopResult>& result)
    {
        for (const Diagnostic& diagnostic : result.diagnostics)
        {
            if (isErrorSeverity(diagnostic.severity))
            {
                return !diagnostic.userMessage.isEmpty()
                    ? diagnostic.userMessage : diagnostic.technicalDetail;
            }
        }
        if (!result.succeeded() && !result.diagnostics.isEmpty())
        {
            const Diagnostic& diagnostic = result.diagnostics.front();
            return !diagnostic.userMessage.isEmpty()
                ? diagnostic.userMessage : diagnostic.technicalDetail;
        }
        return {};
    }
}

QString describeRotaryPathItems(const QVector<CadItem*>& items)
{
    QMap<QString, int> counts;
    int validCount = 0;
    for (const CadItem* item : items)
    {
        if (item == nullptr)
        {
            continue;
        }
        ++validCount;
        ++counts[entityTypeLabel(item->m_type)];
    }

    QStringList parts{ QStringLiteral("%1 个").arg(validCount) };
    for (auto count = counts.cbegin(); count != counts.cend(); ++count)
    {
        parts.push_back(QStringLiteral("%1=%2").arg(count.key()).arg(count.value()));
    }
    return parts.join(QLatin1Char(' '));
}

RotaryPathTopology::RotaryPathTopology
(
    const QVector<CadItem*>& items,
    const RotaryPathTopologyTolerance& tolerance
)
    : m_context(createOperationContext(QStringLiteral("BuildRotaryPathTopology")))
{
    for (CadItem* item : items)
    {
        if (item != nullptr && item->m_entityId != 0U)
        {
            m_itemById.emplace(item->m_entityId, item);
            m_idByItem.emplace(item, item->m_entityId);
        }
    }

    LegacyCadItemTopologyAdapter adapter;
    OperationResult<cadcam::topology::TopologyInput> adapted =
        adapter.convert(items, tolerance, m_context);
    m_diagnostics += adapted.diagnostics;
    if (!adapted.succeeded() || !adapted.value.has_value())
    {
        m_status = adapted.status;
        return;
    }

    TaskContext taskContext;
    taskContext.operationContext = m_context;
    cadcam::topology::PathTopologyBuilder builder;
    OperationResult<cadcam::topology::PathTopology> built =
        builder.build(*adapted.value, tolerance, taskContext);
    m_diagnostics += built.diagnostics;
    if (!built.succeeded() || !built.value.has_value())
    {
        m_status = built.status;
        return;
    }

    m_topology = std::move(*built.value);
    m_status = built.status;
    m_records.reserve(static_cast<qsizetype>(m_topology->records().size()));
    for (const TopologyPathRecord& coreRecord : m_topology->records())
    {
        const auto item = m_itemById.find(coreRecord.entityId);
        if (item == m_itemById.end())
        {
            addMappingDiagnostic
                (coreRecord.entityId, QStringLiteral("core record EntityId has no CadItem mapping"));
            m_topology.reset();
            m_records.clear();
            return;
        }

        RotaryPathTopologyRecord record;
        record.sourceItem = item->second;
        record.sourceItemIndex = static_cast<int>(coreRecord.sourceIndex);
        record.semanticallyClosed = coreRecord.semanticallyClosed;
        record.points.reserve(static_cast<qsizetype>(coreRecord.points.size()));
        for (const Vector3d& point : coreRecord.points)
        {
            record.points.push_back(toLegacyPoint(point));
        }
        m_records.push_back(std::move(record));
    }
}

const QVector<RotaryPathTopologyRecord>& RotaryPathTopology::records() const
{
    return m_records;
}

std::vector<int> RotaryPathTopology::itemComponentIds
(const QVector<CadItem*>& subset) const
{
    if (!m_topology.has_value())
    {
        return {};
    }
    if (subset.isEmpty())
    {
        return m_topology->componentIds();
    }

    const std::vector<EntityId> subsetIds = entityIds(subset);
    const std::vector<int> coreComponents = m_topology->componentIds(subsetIds);
    const std::set<EntityId> requested(subsetIds.begin(), subsetIds.end());
    std::map<EntityId, int> componentById;
    std::size_t componentIndex = 0U;
    for (const TopologyPathRecord& record : m_topology->records())
    {
        if (requested.count(record.entityId) != 0U
            && componentIndex < coreComponents.size())
        {
            componentById.emplace(record.entityId, coreComponents[componentIndex++]);
        }
    }

    std::map<int, int> compatibilityComponentByCore;
    std::vector<int> result;
    result.reserve(subsetIds.size());
    for (EntityId entityId : subsetIds)
    {
        const auto component = componentById.find(entityId);
        if (component == componentById.end())
        {
            continue;
        }
        const auto compatibilityComponent = compatibilityComponentByCore.emplace
        (
            component->second,
            static_cast<int>(compatibilityComponentByCore.size())
        );
        result.push_back(compatibilityComponent.first->second);
    }
    return result;
}

bool RotaryPathTopology::itemsDirectlyConnected(CadItem* left, CadItem* right) const
{
    if (!m_topology.has_value())
    {
        return false;
    }
    const auto leftId = m_idByItem.find(left);
    const auto rightId = m_idByItem.find(right);
    return leftId != m_idByItem.end() && rightId != m_idByItem.end()
        && m_topology->directlyConnected(leftId->second, rightId->second);
}

RotaryPathLoopResult RotaryPathTopology::extractSeededLoop
(
    const QVector<CadItem*>& seedItems,
    QVector<CadItem*>* expandedItems
) const
{
    if (!m_topology.has_value())
    {
        RotaryPathLoopResult result;
        result.errorMessage = m_diagnostics.isEmpty()
            ? QStringLiteral("拓扑尚未成功构建。")
            : (!m_diagnostics.front().userMessage.isEmpty()
                ? m_diagnostics.front().userMessage : m_diagnostics.front().technicalDetail);
        return result;
    }

    const std::vector<EntityId> seeds = entityIds(seedItems);
    const bool everySeedMapped = std::find(seeds.begin(), seeds.end(), 0U) == seeds.end();
    QVector<CadItem*> expanded;
    if (everySeedMapped && !seeds.empty())
    {
        const std::vector<int> components = m_topology->componentIds();
        std::map<EntityId, int> componentById;
        const std::vector<TopologyPathRecord>& coreRecords = m_topology->records();
        for (std::size_t index = 0; index < coreRecords.size(); ++index)
        {
            componentById.emplace(coreRecords[index].entityId, components[index]);
        }
        std::set<int> seedComponents;
        for (EntityId seed : seeds)
        {
            const auto component = componentById.find(seed);
            if (component != componentById.end())
            {
                seedComponents.insert(component->second);
            }
        }
        for (const TopologyPathRecord& record : coreRecords)
        {
            const auto component = componentById.find(record.entityId);
            const auto item = m_itemById.find(record.entityId);
            if (component != componentById.end() && item != m_itemById.end()
                && seedComponents.count(component->second) != 0U)
            {
                expanded.push_back(item->second);
            }
        }
    }

    const OperationResult<TopologyLoopResult> coreResult =
        m_topology->extractSeededLoop(seeds);
    RotaryPathLoopResult result = mapLoopResult(coreResult);
    for (CadItem* item : result.usedItems)
    {
        if (!expanded.contains(item))
        {
            expanded.push_back(item);
        }
    }
    if (expandedItems != nullptr && everySeedMapped)
    {
        *expandedItems = expanded;
    }
    return result;
}

RotaryPathLoopResult RotaryPathTopology::extractBestLoop
(
    const QVector<CadItem*>& candidateItems,
    const QVector<CadItem*>& preferredItems
) const
{
    if (!m_topology.has_value())
    {
        RotaryPathLoopResult result;
        result.errorMessage = m_diagnostics.isEmpty()
            ? QStringLiteral("拓扑尚未成功构建。")
            : (!m_diagnostics.front().userMessage.isEmpty()
                ? m_diagnostics.front().userMessage : m_diagnostics.front().technicalDetail);
        return result;
    }
    return mapLoopResult(m_topology->extractBestLoop
        (entityIds(candidateItems), entityIds(preferredItems)));
}

OperationStatus RotaryPathTopology::status() const
{
    return m_status;
}

const QVector<Diagnostic>& RotaryPathTopology::diagnostics() const
{
    return m_diagnostics;
}

std::vector<EntityId> RotaryPathTopology::entityIds
(const QVector<CadItem*>& items) const
{
    std::vector<EntityId> result;
    result.reserve(static_cast<std::size_t>(items.size()));
    for (CadItem* item : items)
    {
        const auto id = m_idByItem.find(item);
        if (id != m_idByItem.end())
        {
            result.push_back(id->second);
        }
        else
        {
            result.push_back(0U);
        }
    }
    return result;
}

RotaryPathLoopResult RotaryPathTopology::mapLoopResult
(const OperationResult<TopologyLoopResult>& coreResult) const
{
    RotaryPathLoopResult result;
    m_diagnostics += coreResult.diagnostics;
    m_status = coreResult.status;
    result.valid = coreResult.succeeded() && coreResult.value.has_value();
    result.errorMessage = compatibilityError(coreResult);
    if (!coreResult.value.has_value())
    {
        return result;
    }

    const TopologyLoopResult& core = *coreResult.value;
    result.connectedLoop = core.connectedLoop;
    result.approximatelyClosed = core.approximatelyClosed;
    result.closureGap = core.closureGap;
    result.connectedComponentCount = core.connectedComponentCount;
    result.openNodeCount = core.openNodeCount;
    result.branchNodeCount = core.branchNodeCount;
    result.ignoredBranchItemCount = core.ignoredBranchRecordCount;
    result.orderedPath.reserve(static_cast<qsizetype>(core.orderedPath.size()));
    for (const Vector3d& point : core.orderedPath)
    {
        result.orderedPath.push_back(toLegacyPoint(point));
    }
    for (EntityId entityId : core.usedEntityIds)
    {
        const auto item = m_itemById.find(entityId);
        if (item == m_itemById.end())
        {
            addMappingDiagnostic(entityId, QStringLiteral("used EntityId has no CadItem mapping"));
            result.valid = false;
            result.connectedLoop = false;
            result.errorMessage = QStringLiteral("拓扑结果无法映射回原图元。");
            return result;
        }
        result.usedItems.push_back(item->second);
    }
    for (EntityId entityId : core.ignoredBranchEntityIds)
    {
        const auto item = m_itemById.find(entityId);
        if (item == m_itemById.end())
        {
            addMappingDiagnostic(entityId, QStringLiteral("ignored EntityId has no CadItem mapping"));
            result.valid = false;
            result.connectedLoop = false;
            result.errorMessage = QStringLiteral("拓扑结果无法映射回原图元。");
            return result;
        }
        result.ignoredBranchItems.push_back(item->second);
    }
    result.ignoredBranchItemCount = result.ignoredBranchItems.size();
    return result;
}

void RotaryPathTopology::addMappingDiagnostic(EntityId entityId, const QString& detail) const
{
    Diagnostic diagnostic;
    diagnostic.code = DiagnosticCode::TopologyResultMappingFailure;
    diagnostic.severity = DiagnosticSeverity::Error;
    diagnostic.component = QStringLiteral("RotaryPathTopology");
    diagnostic.operation = QStringLiteral("MapCoreTopologyResult");
    diagnostic.stage = QStringLiteral("EntityIdToCadItem");
    diagnostic.userMessage = QStringLiteral("拓扑结果无法映射回原图元。");
    diagnostic.technicalDetail = detail;
    diagnostic.correlationId = m_context.correlationId;
    diagnostic.entityId = entityId;
    diagnostic.context.insert
        (QStringLiteral("entityId"), static_cast<qulonglong>(entityId));
    m_diagnostics.push_back(diagnostic);
    m_status = OperationStatus::Failed;
}
