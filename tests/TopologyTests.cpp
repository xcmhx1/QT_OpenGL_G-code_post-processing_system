#include "TopologyTests.h"

#include "CadDocument.h"
#include "CadItem.h"
#include "RotaryPathTopology.h"
#include "application/geometry/DocumentGeometrySnapshotBuilder.h"
#include "application/geometry/GeometrySnapshotCompiler.h"
#include "application/topology/GeometrySnapshotTopologyAdapter.h"
#include "compatibility/legacy/LegacyCadItemTopologyAdapter.h"
#include "compatibility/legacy/LegacyTopologyParityVerifier.h"
#include "core/topology/PathTopology.h"

#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QFileInfo>

#include <algorithm>
#include <iostream>
#include <memory>

namespace
{
    using cadcam::geometry::EntityId;
    using cadcam::geometry::SourceGeometryKind;
    using cadcam::geometry::Vector3d;
    using cadcam::topology::PathTopology;
    using cadcam::topology::PathTopologyBuilder;
    using cadcam::topology::PathTopologyTolerance;
    using cadcam::topology::TopologyInput;
    using cadcam::topology::TopologyLoopResult;
    using cadcam::topology::TopologyPathRecord;

    int failures = 0;

    class EmptyPathCadItem final : public CadItem
    {
    public:
        explicit EmptyPathCadItem(DRW_Entity* entity)
            : CadItem(entity)
        {
        }

        void buildGeometryDatay() override
        {
        }

        void rebuildRawPathPoints3D() override
        {
            m_rawPathPoints3D.clear();
        }
    };

    void check(bool condition, const char* name)
    {
        if (!condition)
        {
            ++failures;
            std::cerr << "FAILED: " << name << '\n';
        }
    }

    TaskContext task(const QString& name)
    {
        TaskContext value;
        value.operationContext = createOperationContext(name);
        return value;
    }

    TopologyPathRecord record
    (
        std::size_t sourceIndex,
        EntityId entityId,
        std::initializer_list<Vector3d> points,
        bool closed = false,
        SourceGeometryKind kind = SourceGeometryKind::Line
    )
    {
        TopologyPathRecord value;
        value.sourceIndex = sourceIndex;
        value.entityId = entityId;
        value.sourceKind = kind;
        value.points.assign(points.begin(), points.end());
        value.semanticallyClosed = closed;
        if (closed && value.points.size() >= 3U)
        {
            value.points.push_back(value.points.front());
        }
        return value;
    }

    OperationResult<PathTopology> build
    (
        std::vector<TopologyPathRecord> records,
        const TaskContext& taskContext = task(QStringLiteral("topology-test"))
    )
    {
        TopologyInput input;
        input.contentRevision = 1U;
        input.records = std::move(records);
        return PathTopologyBuilder{}.build(input, PathTopologyTolerance{}, taskContext);
    }

    std::vector<TopologyPathRecord> rectangle
    (EntityId firstId = 1U, double x = 0.0)
    {
        return
        {
            record(0U, firstId, { { x, 0, 0 }, { x, 10, 0 } }),
            record(1U, firstId + 1U, { { x, 10, 0 }, { x, 10, 10 } }),
            record(2U, firstId + 2U, { { x, 10, 10 }, { x, 0, 10 } }),
            record(3U, firstId + 3U, { { x, 0, 10 }, { x, 0, 0 } })
        };
    }

    QByteArray orderedPathDigest(const std::vector<Vector3d>& path)
    {
        QStringList parts;
        for (const Vector3d& point : path)
        {
            parts.push_back(QStringLiteral("%1,%2,%3")
                .arg(point.x, 0, 'g', 17)
                .arg(point.y, 0, 'g', 17)
                .arg(point.z, 0, 'g', 17));
        }
        return QCryptographicHash::hash
            (parts.join(QLatin1Char(';')).toUtf8(), QCryptographicHash::Sha256).toHex();
    }

    QJsonArray idsJson(const std::vector<EntityId>& ids)
    {
        QJsonArray array;
        for (EntityId id : ids)
        {
            array.push_back(static_cast<qint64>(id));
        }
        return array;
    }

    QJsonObject topologyGolden(const PathTopology& topology, const TopologyLoopResult& loop)
    {
        QJsonObject object;
        std::vector<EntityId> recordIds;
        QJsonArray adjacency;
        for (std::size_t index = 0; index < topology.records().size(); ++index)
        {
            recordIds.push_back(topology.records()[index].entityId);
            QJsonArray neighbors;
            for (int neighbor : topology.adjacency()[index])
            {
                neighbors.push_back(static_cast<qint64>
                    (topology.records()[static_cast<std::size_t>(neighbor)].entityId));
            }
            adjacency.push_back(neighbors);
        }
        QJsonArray components;
        for (int component : topology.componentIds())
        {
            components.push_back(component);
        }
        object.insert(QStringLiteral("records"), idsJson(recordIds));
        object.insert(QStringLiteral("adjacency"), adjacency);
        object.insert(QStringLiteral("componentPartition"), components);
        object.insert(QStringLiteral("usedEntityIds"), idsJson(loop.usedEntityIds));
        object.insert(QStringLiteral("ignoredBranchEntityIds"), idsJson(loop.ignoredBranchEntityIds));
        object.insert(QStringLiteral("closureGap"), loop.closureGap);
        object.insert(QStringLiteral("orderedPathDigest"),
            QString::fromLatin1(orderedPathDigest(loop.orderedPath)));
        return object;
    }

    void testConnectivityAndComponents()
    {
        const OperationResult<PathTopology> independent = build
        ({
            record(0U, 1U, { { 0, 0, 0 }, { 0, 10, 0 } }),
            record(1U, 2U, { { 0, 20, 0 }, { 0, 30, 0 } })
        });
        check(independent.succeeded() && independent.value.has_value(), "independent paths build");
        check(independent.value.has_value()
            && independent.value->componentIds() == std::vector<int>({ 0, 1 }),
            "independent paths form two components");

        const OperationResult<PathTopology> connected = build
        ({
            record(0U, 1U, { { 0, 0, 0 }, { 0, 10, 0 } }),
            record(1U, 2U, { { 0, 5, 0 }, { 0, 5, 5 } }),
            record(2U, 3U, { { 0, 4, -1 }, { 0, 6, 1 } }),
            record(3U, 4U, { { 0, 10.8, 0 }, { 0, 20, 0 } })
        });
        check(connected.value->directlyConnected(1U, 2U), "endpoint to path middle");
        check(connected.value->directlyConnected(1U, 3U), "line segment intersection");
        check(connected.value->directlyConnected(1U, 4U), "one millimetre endpoint snap");
    }

    void testLoopsBranchesAndGolden()
    {
        const OperationResult<PathTopology> topology = build(rectangle());
        const OperationResult<TopologyLoopResult> loop = topology.value->extractSeededLoop({ 1U });
        check(loop.succeeded() && loop.value.has_value() && loop.value->connectedLoop,
            "seeded rectangle loop");
        check(loop.value->usedEntityIds == std::vector<EntityId>({ 1, 2, 3, 4 }),
            "rectangle uses all records");
        QFile file(QStringLiteral("tests/golden/topology/basic_section.json"));
        check(file.open(QIODevice::ReadOnly), "topology golden opens");
        const QJsonObject expected = QJsonDocument::fromJson(file.readAll()).object();
        check(topologyGolden(*topology.value, *loop.value) == expected, "topology golden matches");

        std::vector<TopologyPathRecord> approximate = rectangle(10U);
        approximate.front().points.front().z = 0.5;
        const OperationResult<PathTopology> approximateTopology = build(std::move(approximate));
        const OperationResult<TopologyLoopResult> approximateLoop =
            approximateTopology.value->extractBestLoop({});
        check(approximateLoop.value.has_value() && approximateLoop.value->approximatelyClosed
            && approximateLoop.value->closureGap > 0.0
            && approximateLoop.value->closureGap <= 1.0,
            "approximately closed loop");

        std::vector<TopologyPathRecord> branched = rectangle(20U);
        branched.push_back(record(4U, 24U, { { 0, 5, 0 }, { 0, 5, -5 } }));
        const OperationResult<PathTopology> branchTopology = build(std::move(branched));
        const OperationResult<TopologyLoopResult> branchLoop =
            branchTopology.value->extractSeededLoop({ 20U });
        check(branchLoop.value.has_value() && branchLoop.value->connectedLoop
            && branchLoop.value->ignoredBranchEntityIds == std::vector<EntityId>({ 24U }),
            "T branch is peeled");
    }

    void testClosedKindsInclinedAndMixedLoops()
    {
        const SourceGeometryKind closedKinds[] =
        {
            SourceGeometryKind::Circle,
            SourceGeometryKind::Ellipse,
            SourceGeometryKind::Polyline,
            SourceGeometryKind::Spline
        };
        for (std::size_t index = 0; index < std::size(closedKinds); ++index)
        {
            const EntityId id = 40U + static_cast<EntityId>(index);
            const OperationResult<PathTopology> topology = build
            ({
                record(index, id,
                    { { 0, 0, 0 }, { 0, 10, 0 }, { 0, 10, 10 }, { 0, 0, 10 } },
                    true, closedKinds[index])
            });
            const OperationResult<TopologyLoopResult> loop =
                topology.value->extractSeededLoop({ id });
            check(loop.succeeded() && loop.value.has_value() && loop.value->connectedLoop,
                "semantic closed path kind forms loop");
        }

        const OperationResult<PathTopology> inclined = build
        ({
            record(0U, 50U, { { 0, 0, 0 }, { 2, 10, 0 } }),
            record(1U, 51U, { { 2, 10, 0 }, { 1, 10, 10 } }),
            record(2U, 52U, { { 1, 10, 10 }, { -1, 0, 10 } }),
            record(3U, 53U, { { -1, 0, 10 }, { 0, 0, 0 } })
        });
        const OperationResult<TopologyLoopResult> inclinedLoop =
            inclined.value->extractSeededLoop({ 50U });
        check(inclinedLoop.succeeded() && inclinedLoop.value.has_value()
            && inclinedLoop.value->usedEntityIds.size() == 4U,
            "inclined section loop");

        const OperationResult<PathTopology> nPath = build
        ({
            record(0U, 60U, { { 0, 0, 0 }, { 1, 0, 5 }, { -1, 0, 10 } }),
            record(1U, 61U, { { -1, 0, 10 }, { 0, 5, 10 }, { 1, 10, 5 } }),
            record(2U, 62U, { { 1, 10, 5 }, { 0, 10, 0 }, { 0, 0, 0 } })
        });
        const OperationResult<TopologyLoopResult> nLoop = nPath.value->extractBestLoop({});
        check(nLoop.succeeded() && nLoop.value.has_value() && nLoop.value->connectedLoop,
            "multi-value N path loop");

        const OperationResult<PathTopology> mixed = build
        ({
            record(0U, 63U, { { 0, 0, 0 }, { 0, 8, 0 } }, false,
                SourceGeometryKind::Line),
            record(1U, 64U, { { 0, 8, 0 }, { 0, 10, 5 } }, false,
                SourceGeometryKind::Arc),
            record(2U, 65U, { { 0, 10, 5 }, { 0, 8, 10 } }, false,
                SourceGeometryKind::Ellipse),
            record(3U, 66U, { { 0, 8, 10 }, { 0, 0, 10 } }, false,
                SourceGeometryKind::Polyline),
            record(4U, 67U, { { 0, 0, 10 }, { 0, -2, 5 }, { 0, 0, 0 } }, false,
                SourceGeometryKind::Spline)
        });
        const OperationResult<TopologyLoopResult> mixedLoop =
            mixed.value->extractSeededLoop({ 63U });
        check(mixedLoop.succeeded() && mixedLoop.value.has_value()
            && mixedLoop.value->usedEntityIds.size() == 5U,
            "mixed line arc ellipse polyline spline loop");

        std::vector<TopologyPathRecord> bridged = rectangle(70U);
        bridged.push_back(record(4U, 80U, { { 0, 20, 0 }, { 0, 24, 0 } }));
        bridged.push_back(record(5U, 81U, { { 0, 24, 0 }, { 0, 24, 4 } }));
        bridged.push_back(record(6U, 82U, { { 0, 24, 4 }, { 0, 20, 4 } }));
        bridged.push_back(record(7U, 83U, { { 0, 20, 4 }, { 0, 20, 0 } }));
        bridged.push_back(record(8U, 90U, { { 0, 10, 0 }, { 0, 20, 0 } }));
        const OperationResult<PathTopology> bridgeTopology = build(std::move(bridged));
        const OperationResult<TopologyLoopResult> bridgeLoop =
            bridgeTopology.value->extractSeededLoop({ 70U });
        check(bridgeLoop.succeeded() && bridgeLoop.value.has_value()
            && bridgeLoop.value->usedEntityIds == std::vector<EntityId>({ 70, 71, 72, 73 }),
            "seed selects one loop across a bridge");
    }

    GeometrySnapshot makeAdapterSnapshot()
    {
        GeometrySnapshot snapshot;
        snapshot.contentRevision = 9U;

        GeometrySnapshotEntry valid;
        valid.sourceIndex = 0U;
        valid.attributes.entityId = 100U;
        valid.sourceKind = SourceGeometryKind::Polyline;
        valid.status = OperationStatus::Success;
        cadcam::geometry::Path3D path;
        path.sourceKind = SourceGeometryKind::Polyline;
        path.closed = true;
        path.vertices =
        {
            { { 0, 0, 0 }, 0.0 },
            { { 0, 10, 0 }, 1.0 },
            { { 0, 10, 0 }, 1.0 },
            { { 0, 10, 10 }, 2.0 },
            { { 0, 0, 10 }, 3.0 }
        };
        valid.path = std::move(path);
        snapshot.entries.push_back(std::move(valid));

        GeometrySnapshotEntry failed;
        failed.sourceIndex = 1U;
        failed.attributes.entityId = 101U;
        failed.sourceKind = SourceGeometryKind::Arc;
        failed.status = OperationStatus::Failed;
        Diagnostic diagnostic;
        diagnostic.code = DiagnosticCode::TopologyPathUnavailable;
        diagnostic.severity = DiagnosticSeverity::Warning;
        diagnostic.technicalDetail = QStringLiteral("test unavailable path");
        failed.diagnostics.push_back(diagnostic);
        snapshot.entries.push_back(std::move(failed));
        return snapshot;
    }

    void testAdapterCancellationRevisionAndDeterminism()
    {
        GeometrySnapshotTopologyAdapter adapter;
        const PathTopologyTolerance tolerance;
        const GeometrySnapshot snapshot = makeAdapterSnapshot();
        const OperationResult<TopologyInput> converted = adapter.convert
            (snapshot, {}, tolerance, createOperationContext(QStringLiteral("adapter-all")));
        check(converted.status == OperationStatus::PartialSuccess
            && converted.value.has_value() && converted.value->records.size() == 1U,
            "adapter retains valid entries during partial failure");
        check(converted.value.has_value() && converted.value->records.front().points.size() == 5U
            && converted.value->records.front().points.front().x
                == converted.value->records.front().points.back().x
            && converted.value->records.front().points.front().y
                == converted.value->records.front().points.back().y
            && converted.value->records.front().points.front().z
                == converted.value->records.front().points.back().z,
            "adapter removes duplicates and appends closed start");

        const OperationResult<TopologyInput> subset = adapter.convert
            (snapshot, { 100U }, tolerance,
                createOperationContext(QStringLiteral("adapter-subset")));
        check(subset.status == OperationStatus::Success && subset.value.has_value()
            && subset.value->records.size() == 1U,
            "adapter explicit subset excludes unrelated failure");

        std::vector<TopologyPathRecord> many;
        for (std::size_t index = 0; index < 64U; ++index)
        {
            const double y = static_cast<double>(index) * 2.0;
            many.push_back(record(index, 200U + index,
                { { 0, y, 0 }, { 0, y + 1.0, 0 } }));
        }
        CancellationSource cancellation;
        TaskContext cancelledTask = task(QStringLiteral("topology-cancel"));
        cancelledTask.cancellationToken = cancellation.token();
        cancelledTask.progressCallback = [&](const GeometryBuildProgress& progress)
        {
            if (progress.completedCount >= 2U)
            {
                cancellation.cancel();
            }
        };
        const OperationResult<PathTopology> cancelled = build(std::move(many), cancelledTask);
        check(cancelled.status == OperationStatus::Cancelled && !cancelled.diagnostics.isEmpty(),
            "topology build cancellation");

        std::vector<TopologyPathRecord> shuffled = rectangle(300U);
        std::reverse(shuffled.begin(), shuffled.end());
        const OperationResult<PathTopology> first = build(rectangle(300U));
        const OperationResult<PathTopology> second = build(std::move(shuffled));
        const OperationResult<TopologyLoopResult> firstLoop = first.value->extractSeededLoop({ 300U });
        const OperationResult<TopologyLoopResult> secondLoop = second.value->extractSeededLoop({ 300U });
        check(firstLoop.value.has_value() && secondLoop.value.has_value()
            && firstLoop.value->usedEntityIds == secondLoop.value->usedEntityIds
            && orderedPathDigest(firstLoop.value->orderedPath)
                == orderedPathDigest(secondLoop.value->orderedPath),
            "topology result deterministic after input reorder");
        check(snapshot.matchesRevision(9U) && !snapshot.matchesRevision(10U),
            "snapshot revision matching remains available to topology caller");
    }

    std::unique_ptr<DRW_Line> makeDocumentLine
    (
        double x1, double y1, double z1,
        double x2, double y2, double z2
    )
    {
        auto line = std::make_unique<DRW_Line>();
        line->basePoint = DRW_Coord(x1, y1, z1);
        line->secPoint = DRW_Coord(x2, y2, z2);
        return line;
    }

    std::unique_ptr<DRW_Ellipse> makeTopologyEllipse()
    {
        auto ellipse = std::make_unique<DRW_Ellipse>();
        ellipse->basePoint = DRW_Coord(10.0, 30.0, 0.0);
        ellipse->secPoint = DRW_Coord(0.0, 6.0, 0.0);
        ellipse->ratio = 0.5;
        ellipse->staparam = 0.0;
        ellipse->endparam = 6.28318530717958647692;
        ellipse->extPoint = DRW_Coord(1.0, 0.0, 0.0);
        return ellipse;
    }

    std::unique_ptr<DRW_LWPolyline> makeTopologyBulgePolyline()
    {
        auto polyline = std::make_unique<DRW_LWPolyline>();
        polyline->flags = 1;
        polyline->extPoint = DRW_Coord(1.0, 0.0, 0.0);
        const double points[4][2] =
        {
            { 45.0, 0.0 }, { 55.0, 0.0 }, { 55.0, 10.0 }, { 45.0, 10.0 }
        };
        for (int index = 0; index < 4; ++index)
        {
            auto vertex = std::make_shared<DRW_Vertex2D>();
            vertex->x = points[index][0];
            vertex->y = points[index][1];
            vertex->bulge = index == 0 ? 0.25 : 0.0;
            polyline->vertlist.push_back(vertex);
        }
        polyline->vertexnum = static_cast<int>(polyline->vertlist.size());
        return polyline;
    }

    std::unique_ptr<DRW_Spline> makeTopologySpline()
    {
        auto spline = std::make_unique<DRW_Spline>();
        spline->degree = 3;
        spline->flags = 4;
        spline->knotslist = { 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0 };
        spline->weightlist = { 1.0, 0.8, 0.8, 1.0 };
        spline->controllist =
        {
            std::make_shared<DRW_Coord>(20.0, 60.0, 0.0),
            std::make_shared<DRW_Coord>(22.0, 64.0, 1.0),
            std::make_shared<DRW_Coord>(24.0, 56.0, 2.0),
            std::make_shared<DRW_Coord>(26.0, 60.0, 3.0)
        };
        spline->ncontrol = static_cast<dint32>(spline->controllist.size());
        spline->nknots = static_cast<dint32>(spline->knotslist.size());
        return spline;
    }

    void testLegacyAdapterValidationAndOrdering()
    {
        LegacyCadItemTopologyAdapter adapter;
        const PathTopologyTolerance tolerance;
        const OperationContext context =
            createOperationContext(QStringLiteral("legacy-adapter-validation"));

        CadDocument orderedDocument;
        CadItem* first = orderedDocument.appendEntity
            (makeDocumentLine(0, 0, 0, 0, 1, 0));
        CadItem* second = orderedDocument.appendEntity
            (makeDocumentLine(0, 10, 0, 0, 11, 0));
        CadItem* third = orderedDocument.appendEntity
            (makeDocumentLine(0, 20, 0, 0, 21, 0));
        const QVector<CadItem*> ordered{ third, first, second };
        const OperationResult<TopologyInput> converted =
            adapter.convert(ordered, tolerance, context);
        check(converted.status == OperationStatus::Success && converted.value.has_value()
            && converted.value->records.size() == 3U,
            "legacy adapter accepts valid CadItems");
        check(converted.value.has_value()
            && converted.value->records[0].entityId == third->m_entityId
            && converted.value->records[1].entityId == first->m_entityId
            && converted.value->records[2].entityId == second->m_entityId
            && converted.value->records[0].sourceIndex == 0U
            && converted.value->records[2].sourceIndex == 2U,
            "legacy adapter preserves input source order");

        const OperationResult<TopologyInput> nullItem =
            adapter.convert({ nullptr }, tolerance, context);
        check(nullItem.status == OperationStatus::InvalidInput
            && !nullItem.diagnostics.isEmpty()
            && nullItem.diagnostics.front().code
                == DiagnosticCode::LegacyTopologyAdapterFailure,
            "legacy adapter diagnoses null CadItem");

        CadDocument zeroDocument;
        CadItem* zero = zeroDocument.appendEntity
            (makeDocumentLine(0, 0, 0, 0, 1, 0));
        zero->m_entityId = 0U;
        const OperationResult<TopologyInput> zeroId =
            adapter.convert({ zero }, tolerance, context);
        check(zeroId.status == OperationStatus::InvalidInput
            && !zeroId.diagnostics.isEmpty(),
            "legacy adapter diagnoses zero EntityId");

        CadDocument duplicateDocument;
        CadItem* duplicateFirst = duplicateDocument.appendEntity
            (makeDocumentLine(0, 0, 0, 0, 1, 0));
        CadItem* duplicateSecond = duplicateDocument.appendEntity
            (makeDocumentLine(0, 2, 0, 0, 3, 0));
        duplicateSecond->m_entityId = duplicateFirst->m_entityId;
        const OperationResult<TopologyInput> duplicate = adapter.convert
            ({ duplicateFirst, duplicateSecond }, tolerance, context);
        check(duplicate.status == OperationStatus::InvalidInput
            && !duplicate.diagnostics.isEmpty()
            && duplicate.diagnostics.back().code
                == DiagnosticCode::DuplicateTopologyEntityId,
            "legacy adapter diagnoses duplicate EntityId");

        DRW_Line emptyEntity;
        EmptyPathCadItem emptyItem(&emptyEntity);
        emptyItem.m_entityId = 900U;
        const OperationResult<TopologyInput> emptyPath =
            adapter.convert({ &emptyItem }, tolerance, context);
        check(emptyPath.status == OperationStatus::InvalidInput
            && !emptyPath.diagnostics.isEmpty(),
            "legacy adapter diagnoses empty process path");
    }

    void testLegacyWrapperPublicApi()
    {
        CadDocument document;
        QVector<CadItem*> items;
        items.push_back(document.appendEntity(makeDocumentLine(0, 0, 0, 0, 10, 0)));
        items.push_back(document.appendEntity(makeDocumentLine(0, 10, 0, 0, 10, 10)));
        items.push_back(document.appendEntity(makeDocumentLine(0, 10, 10, 0, 0, 10)));
        items.push_back(document.appendEntity(makeDocumentLine(0, 0, 10, 0, 0, 0)));
        CadItem* branch = document.appendEntity(makeDocumentLine(0, 5, 0, 0, 5, -5));
        items.push_back(branch);
        CadItem* remote = document.appendEntity(makeDocumentLine(20, 30, 0, 20, 31, 0));
        items.push_back(remote);

        RotaryPathTopology topology(items, PathTopologyTolerance{});
        check(topology.status() == OperationStatus::Success
            && topology.records().size() == items.size(),
            "legacy public wrapper builds through core topology");
        check(topology.records()[0].sourceItem == items[0]
            && topology.records()[4].sourceItem == branch
            && topology.records()[4].sourceItemIndex == 4,
            "legacy records map EntityId back to CadItem");
        check(topology.itemsDirectlyConnected(items[0], items[1])
            && !topology.itemsDirectlyConnected(items[0], remote),
            "legacy direct connectivity delegates to core");
        const std::vector<int> subsetComponents =
            topology.itemComponentIds({ remote, items[0] });
        check(subsetComponents == std::vector<int>({ 0, 1 }),
            "legacy subset component output preserves subset order");
        const std::vector<int> inducedSubsetComponents =
            topology.itemComponentIds({ items[0], items[2] });
        check(inducedSubsetComponents == std::vector<int>({ 0, 1 }),
            "legacy subset components exclude paths through omitted items");

        QVector<CadItem*> expanded;
        const RotaryPathLoopResult seeded = topology.extractSeededLoop
            ({ items[0] }, &expanded);
        check(seeded.valid && seeded.connectedLoop && seeded.usedItems.size() == 4
            && seeded.ignoredBranchItems == QVector<CadItem*>({ branch }),
            "legacy seeded loop maps used and ignored items");
        check(expanded.size() == 5 && expanded.contains(branch)
            && !expanded.contains(remote),
            "legacy seeded loop preserves expanded connected items");

        CadDocument preferredDocument;
        QVector<CadItem*> preferredItems;
        for (double x : { 0.0, 20.0 })
        {
            preferredItems.push_back(preferredDocument.appendEntity
                (makeDocumentLine(x, 0, 0, x, 10, 0)));
            preferredItems.push_back(preferredDocument.appendEntity
                (makeDocumentLine(x, 10, 0, x, 10, 10)));
            preferredItems.push_back(preferredDocument.appendEntity
                (makeDocumentLine(x, 10, 10, x, 0, 10)));
            preferredItems.push_back(preferredDocument.appendEntity
                (makeDocumentLine(x, 0, 10, x, 0, 0)));
        }
        RotaryPathTopology preferredTopology(preferredItems, PathTopologyTolerance{});
        const RotaryPathLoopResult preferred = preferredTopology.extractBestLoop
            (preferredItems, { preferredItems[4] });
        check(preferred.valid && preferred.usedItems.contains(preferredItems[4])
            && !preferred.usedItems.contains(preferredItems[0]),
            "legacy best loop honors preferred items");
    }

    void testLegacyAdapterProcessSemanticsAndTypes()
    {
        LegacyCadItemTopologyAdapter adapter;
        const PathTopologyTolerance tolerance;
        const OperationContext context =
            createOperationContext(QStringLiteral("legacy-adapter-process-semantics"));

        CadDocument lineDocument;
        CadItem* line = lineDocument.appendEntity
            (makeDocumentLine(0, 0, 0, 10, 0, 0));
        const OperationResult<TopologyInput> forward =
            adapter.convert({ line }, tolerance, context);
        line->m_isReverse = true;
        const OperationResult<TopologyInput> reverse =
            adapter.convert({ line }, tolerance, context);
        check(forward.value.has_value() && reverse.value.has_value()
            && forward.value->records.front().points.front().x
                == reverse.value->records.front().points.back().x
            && forward.value->records.front().points.back().x
                == reverse.value->records.front().points.front().x,
            "legacy adapter preserves reverse process direction");

        CadDocument circleDocument;
        auto circle = std::make_unique<DRW_Circle>();
        circle->basePoint = DRW_Coord(0.0, 0.0, 0.0);
        circle->radious = 5.0;
        circle->extPoint = DRW_Coord(0.0, 0.0, 1.0);
        CadItem* circleItem = circleDocument.appendEntity(std::move(circle));
        const OperationResult<TopologyInput> defaultCircle =
            adapter.convert({ circleItem }, tolerance, context);
        circleItem->m_hasCustomProcessStart = true;
        circleItem->m_processStartParameter = 0.0;
        circleItem->m_isReverse = true;
        const OperationResult<TopologyInput> customCircle =
            adapter.convert({ circleItem }, tolerance, context);
        check(defaultCircle.value.has_value() && customCircle.value.has_value()
            && customCircle.value->records.front().semanticallyClosed
            && (defaultCircle.value->records.front().points.front().x
                    != customCircle.value->records.front().points.front().x
                || defaultCircle.value->records.front().points.front().y
                    != customCircle.value->records.front().points.front().y),
            "legacy adapter preserves custom closed start and reverse semantics");

        CadDocument mixedDocument;
        mixedDocument.appendEntity(makeTopologyEllipse());
        mixedDocument.appendEntity(makeTopologyBulgePolyline());
        mixedDocument.appendEntity(makeTopologySpline());
        DocumentGeometrySnapshotBuilder sourceBuilder;
        const OperationResult<GeometrySourceSnapshot> source = sourceBuilder.capture
            (mixedDocument, createOperationContext(QStringLiteral("topology-mixed-capture")));
        GeometrySnapshotCompiler compiler;
        cadcam::geometry::SamplingPolicy compatibilityPolicy;
        compatibilityPolicy.chordTolerance = 0.0;
        compatibilityPolicy.singlePrecisionEvaluation = true;
        const OperationResult<GeometrySnapshot> snapshot = compiler.compile
        (
            *source.value,
            compatibilityPolicy,
            GeometryExecutionMode::Serial,
            task(QStringLiteral("topology-mixed-compile"))
        );
        LegacyTopologyParityOptions options;
        for (const GeometrySnapshotEntry& entry : snapshot.value->entries)
        {
            options.candidates.push_back(entry.attributes.entityId);
        }
        LegacyTopologyParityVerifier verifier;
        const OperationResult<LegacyTopologyParityReport> parity = verifier.verify
        (
            mixedDocument, *snapshot.value, options, tolerance,
            task(QStringLiteral("topology-mixed-adapter-parity"))
        );
        check(parity.value.has_value() && parity.value->recordsEquivalent
            && parity.value->adjacencyEquivalent && parity.value->componentsEquivalent
            && parity.value->bestLoopEquivalent && parity.value->exactEquivalent,
            "ellipse bulge polyline and spline adapter parity");
    }

    void testLegacyWrapperApproximateClosureAndSourceBoundary()
    {
        CadDocument document;
        QVector<CadItem*> items;
        items.push_back(document.appendEntity(makeDocumentLine(0, 0, 0.5, 0, 10, 0)));
        items.push_back(document.appendEntity(makeDocumentLine(0, 10, 0, 0, 10, 10)));
        items.push_back(document.appendEntity(makeDocumentLine(0, 10, 10, 0, 0, 10)));
        items.push_back(document.appendEntity(makeDocumentLine(0, 0, 10, 0, 0, 0)));
        RotaryPathTopology topology(items, PathTopologyTolerance{});
        const RotaryPathLoopResult loop = topology.extractSeededLoop({ items[0] });
        bool hasApproximateDiagnostic = false;
        for (const Diagnostic& diagnostic : topology.diagnostics())
        {
            hasApproximateDiagnostic = hasApproximateDiagnostic
                || diagnostic.code == DiagnosticCode::TopologyApproximateClosure;
        }
        check(loop.valid && loop.approximatelyClosed && hasApproximateDiagnostic,
            "legacy wrapper exposes approximate closure diagnostic");

        CadDocument openDocument;
        CadItem* openItem = openDocument.appendEntity
            (makeDocumentLine(0, 0, 0, 0, 10, 0));
        RotaryPathTopology openTopology({ openItem }, PathTopologyTolerance{});
        const RotaryPathLoopResult openLoop = openTopology.extractBestLoop({ openItem });
        check(!openLoop.valid && !openLoop.errorMessage.isEmpty(),
            "legacy wrapper maps core failure diagnostic to error message");

        const QString sourcePath = QFileInfo(QString::fromUtf8(__FILE__))
            .absoluteDir().absoluteFilePath(QStringLiteral("../src/RotaryPathTopology.cpp"));
        QFile sourceFile(sourcePath);
        check(sourceFile.open(QIODevice::ReadOnly), "legacy topology wrapper source opens");
        const QByteArray sourceText = sourceFile.readAll();
        const QByteArray forbidden[] =
        {
            "pathsConnected", "segmentSegmentDistance", "enumerateSimpleCycles",
            "DisjointSet", "fitTopologyPlane"
        };
        for (const QByteArray& symbol : forbidden)
        {
            check(!sourceText.contains(symbol), "legacy topology source has no core algorithm copy");
        }
    }

    void testLegacyParity()
    {
        CadDocument document;
        document.appendEntity(makeDocumentLine(0, 0, 0, 0, 10, 0));
        document.appendEntity(makeDocumentLine(0, 10, 0, 0, 10, 10));
        document.appendEntity(makeDocumentLine(0, 10, 10, 0, 0, 10));
        document.appendEntity(makeDocumentLine(0, 0, 10, 0, 0, 0));

        DocumentGeometrySnapshotBuilder sourceBuilder;
        const OperationResult<GeometrySourceSnapshot> source = sourceBuilder.capture
            (document, createOperationContext(QStringLiteral("topology-parity-capture")));
        GeometrySnapshotCompiler compiler;
        const OperationResult<GeometrySnapshot> snapshot = compiler.compile
        (
            *source.value,
            cadcam::geometry::SamplingPolicy{},
            GeometryExecutionMode::Serial,
            task(QStringLiteral("topology-parity-compile"))
        );
        LegacyTopologyParityOptions options;
        for (const GeometrySnapshotEntry& entry : snapshot.value->entries)
        {
            options.candidates.push_back(entry.attributes.entityId);
        }
        options.seeds.push_back(options.candidates.front());
        LegacyTopologyParityVerifier verifier;
        const OperationResult<LegacyTopologyParityReport> parity = verifier.verify
        (
            document,
            *snapshot.value,
            options,
            PathTopologyTolerance{},
            task(QStringLiteral("topology-parity-verify"))
        );
        check(parity.status == OperationStatus::Success && parity.value.has_value()
            && parity.value->equivalent,
            "legacy topology shadow parity");
        check(parity.value.has_value() && parity.value->recordsEquivalent
            && parity.value->adjacencyEquivalent && parity.value->componentsEquivalent
            && parity.value->seededLoopEquivalent && parity.value->bestLoopEquivalent,
            "legacy topology parity covers records graph and loops");
        check(parity.value.has_value() && parity.value->exactEquivalent,
            "legacy topology ordered path is exactly equivalent");
        check(parity.value.has_value() && parity.value->maximumPointDistance <= 1.0e-6,
            "legacy topology parity maximum error");
    }
}

int runTopologyTests()
{
    testConnectivityAndComponents();
    testLoopsBranchesAndGolden();
    testClosedKindsInclinedAndMixedLoops();
    testAdapterCancellationRevisionAndDeterminism();
    testLegacyAdapterValidationAndOrdering();
    testLegacyWrapperPublicApi();
    testLegacyAdapterProcessSemanticsAndTypes();
    testLegacyWrapperApproximateClosureAndSourceBoundary();
    testLegacyParity();
    return failures;
}
