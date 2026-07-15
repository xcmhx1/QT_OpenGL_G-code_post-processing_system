#include "compatibility/legacy/LegacyTopologyParityVerifier.h"

#include "cad/document/CadDocument.h"
#include "cad/items/CadItem.h"
#include "application/machining/RotaryPathTopology.h"
#include "application/topology/GeometrySnapshotTopologyAdapter.h"

#include <QThread>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace
{
    using cadcam::geometry::EntityId;
    using cadcam::geometry::Vector3d;
    using cadcam::topology::PathTopology;
    using cadcam::topology::TopologyLoopResult;

    Vector3d fromLegacy(const QVector3D& point)
    {
        return
        {
            static_cast<double>(point.x()),
            static_cast<double>(point.y()),
            static_cast<double>(point.z())
        };
    }

    Vector3d legacyQuantized(const Vector3d& point)
    {
        return
        {
            static_cast<double>(static_cast<float>(point.x)),
            static_cast<double>(static_cast<float>(point.y)),
            static_cast<double>(static_cast<float>(point.z))
        };
    }

    double pointDistance(const Vector3d& left, const Vector3d& right)
    {
        const double dx = left.x - right.x;
        const double dy = left.y - right.y;
        const double dz = left.z - right.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    bool pointsEqual(const Vector3d& left, const Vector3d& right)
    {
        return pointDistance(legacyQuantized(left), legacyQuantized(right)) <= 1.0e-9;
    }

    std::vector<Vector3d> normalizedClosedPath(std::vector<Vector3d> path)
    {
        if (path.size() > 1U && pointsEqual(path.front(), path.back()))
        {
            path.pop_back();
        }
        return path;
    }

    bool exactPathEquivalent
    (
        const std::vector<Vector3d>& left,
        const std::vector<Vector3d>& right
    )
    {
        if (left.size() != right.size())
        {
            return false;
        }
        for (std::size_t index = 0; index < left.size(); ++index)
        {
            if (!pointsEqual(left[index], right[index]))
            {
                return false;
            }
        }
        return true;
    }

    bool cyclicPathEquivalent
    (
        const std::vector<Vector3d>& leftPath,
        const std::vector<Vector3d>& rightPath,
        bool reverse
    )
    {
        const std::vector<Vector3d> left = normalizedClosedPath(leftPath);
        std::vector<Vector3d> right = normalizedClosedPath(rightPath);
        if (left.size() != right.size() || left.empty())
        {
            return left.empty() && right.empty();
        }
        if (reverse)
        {
            std::reverse(right.begin(), right.end());
        }
        for (std::size_t offset = 0; offset < right.size(); ++offset)
        {
            bool equal = true;
            for (std::size_t index = 0; index < left.size(); ++index)
            {
                if (!pointsEqual(left[index], right[(index + offset) % right.size()]))
                {
                    equal = false;
                    break;
                }
            }
            if (equal)
            {
                return true;
            }
        }
        return false;
    }

    std::vector<EntityId> legacyIds
    (
        const QVector<CadItem*>& items
    )
    {
        std::vector<EntityId> result;
        result.reserve(static_cast<std::size_t>(items.size()));
        for (const CadItem* item : items)
        {
            if (item != nullptr)
            {
                result.push_back(item->m_entityId);
            }
        }
        return result;
    }

    std::vector<Vector3d> legacyPoints(const QVector<QVector3D>& points)
    {
        std::vector<Vector3d> result;
        result.reserve(static_cast<std::size_t>(points.size()));
        for (const QVector3D& point : points)
        {
            result.push_back(fromLegacy(point));
        }
        return result;
    }

    bool idSetsEqual(std::vector<EntityId> left, std::vector<EntityId> right)
    {
        std::sort(left.begin(), left.end());
        std::sort(right.begin(), right.end());
        return left == right;
    }

    bool loopEquivalent
    (
        const RotaryPathLoopResult& legacy,
        const OperationResult<TopologyLoopResult>& current
    )
    {
        if (!current.value.has_value())
        {
            return !legacy.valid;
        }
        const TopologyLoopResult& value = *current.value;
        return legacy.valid == value.connectedLoop
            && legacy.connectedLoop == value.connectedLoop
            && std::abs(legacy.maximumJoinGap - value.maximumJoinGap) <= 1.0e-9
            && legacy.connectedComponentCount == value.connectedComponentCount
            && legacy.openNodeCount == value.openNodeCount
            && legacy.branchNodeCount == value.branchNodeCount
            && idSetsEqual(legacyIds(legacy.usedItems), value.usedEntityIds)
            && idSetsEqual(legacyIds(legacy.ignoredBranchItems), value.ignoredBranchEntityIds)
            && (exactPathEquivalent(legacyPoints(legacy.orderedPath), value.orderedPath)
                || cyclicPathEquivalent(legacyPoints(legacy.orderedPath), value.orderedPath, false)
                || cyclicPathEquivalent(legacyPoints(legacy.orderedPath), value.orderedPath, true));
    }

    Diagnostic parityDiagnostic
    (
        const OperationContext& context,
        const QString& detail,
        std::size_t recordCount,
        const cadcam::topology::PathTopologyTolerance& tolerance
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = DiagnosticCode::TopologyParityMismatch;
        diagnostic.severity = DiagnosticSeverity::Warning;
        diagnostic.component = QStringLiteral("LegacyTopologyParityVerifier");
        diagnostic.operation = QStringLiteral("VerifyTopologyParity");
        diagnostic.stage = QStringLiteral("CompareLegacyAndCore");
        diagnostic.userMessage = QStringLiteral("两种拓扑输入适配结果存在差异。");
        diagnostic.technicalDetail = detail;
        diagnostic.correlationId = context.correlationId;
        diagnostic.context.insert(QStringLiteral("entityId"), static_cast<qulonglong>(0U));
        diagnostic.context.insert(QStringLiteral("sourceIndex"), static_cast<qulonglong>(0U));
        diagnostic.context.insert
            (QStringLiteral("recordCount"), static_cast<qulonglong>(recordCount));
        diagnostic.context.insert(QStringLiteral("nodeCount"), static_cast<qulonglong>(0U));
        diagnostic.context.insert(QStringLiteral("edgeCount"), static_cast<qulonglong>(0U));
        diagnostic.context.insert(QStringLiteral("componentCount"), 0);
        diagnostic.context.insert(QStringLiteral("openNodeCount"), 0);
        diagnostic.context.insert(QStringLiteral("branchNodeCount"), 0);
        diagnostic.context.insert(QStringLiteral("maximumJoinGap"), 0.0);
        diagnostic.context.insert
            (QStringLiteral("numericalJoinEpsilon"), tolerance.numericalJoinEpsilon);
        diagnostic.context.insert(QStringLiteral("nodeSnap"), tolerance.nodeSnap);
        diagnostic.context.insert
            (QStringLiteral("intersectionTolerance"), tolerance.intersection);
        return diagnostic;
    }
}

OperationResult<LegacyTopologyParityReport> LegacyTopologyParityVerifier::verify
(
    const CadDocument& document,
    const GeometrySnapshot& snapshot,
    const LegacyTopologyParityOptions& options,
    const cadcam::topology::PathTopologyTolerance& tolerance,
    const TaskContext& taskContext
) const
{
    OperationResult<LegacyTopologyParityReport> result;
    if (QThread::currentThread() != document.thread())
    {
        result.status = OperationStatus::Failed;
        result.addDiagnostic(parityDiagnostic
        (
            taskContext.operationContext,
            QStringLiteral("legacy parity verification must run on document thread"),
            0U,
            tolerance
        ));
        return result;
    }

    GeometrySnapshotTopologyAdapter adapter;
    OperationResult<cadcam::topology::TopologyInput> adapted = adapter.convert
        (snapshot, options.candidates, tolerance, taskContext.operationContext);
    if (!adapted.succeeded() || !adapted.value.has_value())
    {
        result.status = adapted.status;
        result.diagnostics = adapted.diagnostics;
        return result;
    }
    cadcam::topology::PathTopologyBuilder builder;
    OperationResult<PathTopology> built = builder.build
        (*adapted.value, tolerance, taskContext);
    if (!built.succeeded() || !built.value.has_value())
    {
        result.status = built.status;
        result.diagnostics = built.diagnostics;
        return result;
    }
    const PathTopology& topology = *built.value;

    std::map<EntityId, CadItem*> itemById;
    for (const std::unique_ptr<CadItem>& item : document.m_entities)
    {
        if (item != nullptr)
        {
            itemById.emplace(item->m_entityId, item.get());
        }
    }
    QVector<CadItem*> legacyItems;
    for (const cadcam::topology::TopologyPathRecord& record : topology.records())
    {
        const auto item = itemById.find(record.entityId);
        if (item == itemById.end())
        {
            result.status = OperationStatus::Failed;
            Diagnostic diagnostic = parityDiagnostic
            (
                taskContext.operationContext,
                QStringLiteral("snapshot EntityId is not present in CadDocument"),
                topology.records().size(),
                tolerance
            );
            diagnostic.entityId = record.entityId;
            diagnostic.context.insert
                (QStringLiteral("entityId"), static_cast<qulonglong>(record.entityId));
            diagnostic.context.insert
                (QStringLiteral("sourceIndex"), static_cast<qulonglong>(record.sourceIndex));
            result.addDiagnostic(diagnostic);
            return result;
        }
        legacyItems.push_back(item->second);
    }

    const RotaryPathTopologyTolerance legacyTolerance
    {
        tolerance.nodeSnap,
        tolerance.numericalJoinEpsilon,
        tolerance.intersection,
        tolerance.minimumEdgeLength
    };
    RotaryPathTopology legacy(legacyItems, legacyTolerance);
    LegacyTopologyParityReport report;
    report.recordsEquivalent = legacy.records().size()
        == static_cast<qsizetype>(topology.records().size());
    if (report.recordsEquivalent)
    {
        for (int index = 0; index < legacy.records().size(); ++index)
        {
            const RotaryPathTopologyRecord& legacyRecord = legacy.records()[index];
            const cadcam::topology::TopologyPathRecord& currentRecord =
                topology.records()[static_cast<std::size_t>(index)];
            report.recordsEquivalent = report.recordsEquivalent
                && legacyRecord.sourceItem != nullptr
                && legacyRecord.sourceItem->m_entityId == currentRecord.entityId
                && legacyRecord.semanticallyClosed == currentRecord.semanticallyClosed
                && legacyRecord.points.size() == static_cast<qsizetype>(currentRecord.points.size());
            const std::size_t count = std::min
            (
                static_cast<std::size_t>(legacyRecord.points.size()),
                currentRecord.points.size()
            );
            for (std::size_t point = 0; point < count; ++point)
            {
                const Vector3d legacyPoint = fromLegacy
                    (legacyRecord.points[static_cast<qsizetype>(point)]);
                report.maximumPointDistance = std::max
                (
                    report.maximumPointDistance,
                    pointDistance(legacyPoint, currentRecord.points[point])
                );
                report.recordsEquivalent = report.recordsEquivalent
                    && pointsEqual(legacyPoint, currentRecord.points[point]);
            }
        }
    }

    report.adjacencyEquivalent = true;
    for (int left = 0; left < legacyItems.size(); ++left)
    {
        for (int right = 0; right < legacyItems.size(); ++right)
        {
            report.adjacencyEquivalent = report.adjacencyEquivalent
                && legacy.itemsDirectlyConnected(legacyItems[left], legacyItems[right])
                    == topology.directlyConnected
                    (
                        legacyItems[left]->m_entityId,
                        legacyItems[right]->m_entityId
                    );
        }
    }

    const std::vector<int> legacyComponents = legacy.itemComponentIds();
    const std::vector<int> currentComponents = topology.componentIds();
    report.componentsEquivalent = legacyComponents.size() == currentComponents.size();
    for (std::size_t left = 0;
        report.componentsEquivalent && left < legacyComponents.size(); ++left)
    {
        for (std::size_t right = 0; right < legacyComponents.size(); ++right)
        {
            report.componentsEquivalent = report.componentsEquivalent
                && (legacyComponents[left] == legacyComponents[right])
                    == (currentComponents[left] == currentComponents[right]);
        }
    }

    auto mapItems = [&itemById](const std::vector<EntityId>& ids)
    {
        QVector<CadItem*> items;
        for (EntityId entityId : ids)
        {
            const auto item = itemById.find(entityId);
            if (item != itemById.end())
            {
                items.push_back(item->second);
            }
        }
        return items;
    };
    report.seededLoopEquivalent = options.seeds.empty();
    if (!options.seeds.empty())
    {
        const RotaryPathLoopResult legacySeeded = legacy.extractSeededLoop
            (mapItems(options.seeds));
        const OperationResult<TopologyLoopResult> currentSeeded =
            topology.extractSeededLoop(options.seeds);
        report.seededLoopEquivalent = loopEquivalent(legacySeeded, currentSeeded);
    }

    const RotaryPathLoopResult legacyBest = legacy.extractBestLoop
        (mapItems(options.candidates), mapItems(options.preferred));
    const OperationResult<TopologyLoopResult> currentBest = topology.extractBestLoop
        (options.candidates, options.preferred);
    report.bestLoopEquivalent = loopEquivalent(legacyBest, currentBest);
    if (currentBest.value.has_value())
    {
        report.maximumJoinGapDifference = std::abs
            (legacyBest.maximumJoinGap - currentBest.value->maximumJoinGap);
        const std::vector<Vector3d> oldPath = legacyPoints(legacyBest.orderedPath);
        report.exactEquivalent = exactPathEquivalent(oldPath, currentBest.value->orderedPath);
        report.equivalentAfterCyclicRotation = cyclicPathEquivalent
            (oldPath, currentBest.value->orderedPath, false);
        report.equivalentAfterReverseAndRotation = cyclicPathEquivalent
            (oldPath, currentBest.value->orderedPath, true);
    }

    report.equivalent = report.recordsEquivalent
        && report.adjacencyEquivalent
        && report.componentsEquivalent
        && report.seededLoopEquivalent
        && report.bestLoopEquivalent;
    if (!report.equivalent)
    {
        report.diagnostics.push_back(parityDiagnostic
        (
            taskContext.operationContext,
            QStringLiteral("records=%1 adjacency=%2 components=%3 seeded=%4 best=%5")
                .arg(report.recordsEquivalent)
                .arg(report.adjacencyEquivalent)
                .arg(report.componentsEquivalent)
                .arg(report.seededLoopEquivalent)
                .arg(report.bestLoopEquivalent),
            topology.records().size(),
            tolerance
        ));
    }
    report.diagnostics += adapted.diagnostics;
    report.diagnostics += built.diagnostics;
    result.status = report.equivalent
        ? OperationStatus::Success : OperationStatus::PartialSuccess;
    result.diagnostics = report.diagnostics;
    result.value = std::move(report);
    return result;
}
