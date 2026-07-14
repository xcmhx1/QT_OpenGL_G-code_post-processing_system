#include "TopologyTests.h"

#include "CadDocument.h"
#include "CadItem.h"
#include "RotaryCutBoundaryAnalyzer.h"
#include "RotaryPathTopology.h"
#include "RotaryTubeGeometryAnalyzer.h"
#include "application/geometry/DocumentGeometrySnapshotBuilder.h"
#include "application/geometry/GeometrySnapshotCompiler.h"
#include "application/topology/GeometrySnapshotTopologyAdapter.h"
#include "compatibility/legacy/LegacyCadItemTopologyAdapter.h"
#include "compatibility/legacy/LegacyProcessPlanAdapter.h"
#include "compatibility/legacy/LegacyTopologyParityVerifier.h"
#include "core/machining/TubeCutBoundary.h"
#include "core/planning/PlanarProcessPlanBuilder.h"
#include "core/planning/ProcessPlanBuilder.h"
#include "core/topology/PathTopology.h"

#include <QCryptographicHash>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QFileInfo>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>

namespace
{
    using cadcam::geometry::EntityId;
    using cadcam::geometry::SourceGeometryKind;
    using cadcam::geometry::Vector3d;
    using cadcam::machining::TubeCutAnalysis;
    using cadcam::machining::TubeCutBoundaryClassifier;
    using cadcam::machining::TubeCutResult;
    using cadcam::machining::TubeSectionGeometry;
    using cadcam::machining::TubeSectionAnalyzer;
    using cadcam::machining::TubeSectionPolicy;
    using CoreTubeSectionModel = cadcam::machining::TubeSectionModel;
    using cadcam::topology::PathTopology;
    using cadcam::topology::PathTopologyBuilder;
    using cadcam::topology::PathTopologyTolerance;
    using cadcam::topology::TopologyInput;
    using cadcam::topology::TopologyLoopResult;
    using cadcam::topology::TopologyPathRecord;
    using namespace cadcam::planning;

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

    OperationResult<PathTopology> buildWithTolerance
    (
        std::vector<TopologyPathRecord> records,
        const PathTopologyTolerance& tolerance,
        const TaskContext& taskContext = task(QStringLiteral("topology-tolerance-test"))
    )
    {
        TopologyInput input;
        input.contentRevision = 1U;
        input.records = std::move(records);
        return PathTopologyBuilder{}.build(input, tolerance, taskContext);
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
        object.insert(QStringLiteral("maximumJoinGap"), loop.maximumJoinGap);
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

        std::vector<TopologyPathRecord> discontinuous = rectangle(10U);
        discontinuous.front().points.front().z = 0.5;
        const OperationResult<PathTopology> discontinuousTopology =
            build(std::move(discontinuous));
        const OperationResult<TopologyLoopResult> discontinuousLoop =
            discontinuousTopology.value->extractBestLoop({});
        check(discontinuousLoop.status == OperationStatus::Failed
            && discontinuousLoop.value.has_value()
            && !discontinuousLoop.value->connectedLoop
            && std::abs(discontinuousLoop.value->maximumJoinGap - 0.5) <= 1.0e-9
            && !discontinuousLoop.diagnostics.isEmpty()
            && discontinuousLoop.diagnostics.front().code
                == DiagnosticCode::TopologyLoopDiscontinuous,
            "discontinuous loop is rejected with physical gap diagnostic");

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

    void testStrictLoopContinuity()
    {
        const PathTopologyTolerance smallSearch =
            PathTopologyTolerance::fromConnectionTolerance(0.1);
        const PathTopologyTolerance largeSearch =
            PathTopologyTolerance::fromConnectionTolerance(100.0);
        check(smallSearch.numericalJoinEpsilon == 1.0e-5
            && largeSearch.numericalJoinEpsilon == 1.0e-5,
            "numerical join epsilon is independent of connection tolerance");

        PathTopologyTolerance invalidTolerance;
        invalidTolerance.numericalJoinEpsilon = 0.001;
        const OperationResult<PathTopology> invalid =
            buildWithTolerance(rectangle(400U), invalidTolerance);
        check(invalid.status == OperationStatus::InvalidInput,
            "millimetre scale numerical join epsilon is rejected");

        std::vector<TopologyPathRecord> floatingError = rectangle(410U);
        floatingError.front().points.front().z = 5.0e-6;
        const OperationResult<PathTopology> floatingTopology = build(std::move(floatingError));
        const OperationResult<TopologyLoopResult> floatingLoop =
            floatingTopology.value->extractBestLoop({});
        check(floatingLoop.succeeded() && floatingLoop.value.has_value()
            && floatingLoop.value->connectedLoop
            && floatingLoop.value->maximumJoinGap <= 1.0e-5,
            "sub epsilon floating error remains strictly connected");

        const std::array<double, 5> rejectedGaps =
            { 2.0e-5, 0.01, 0.1, 0.5, 1.0 };
        for (double gap : rejectedGaps)
        {
            std::vector<TopologyPathRecord> records = rectangle
                (420U + static_cast<EntityId>(100.0 * gap));
            records.front().points.front().z = gap;
            const OperationResult<PathTopology> topology = build(std::move(records));
            const OperationResult<TopologyLoopResult> loop =
                topology.value->extractBestLoop({});
            bool hasDiscontinuousDiagnostic = false;
            bool diagnosticContextComplete = false;
            for (const Diagnostic& diagnostic : loop.diagnostics)
            {
                hasDiscontinuousDiagnostic = hasDiscontinuousDiagnostic
                    || diagnostic.code == DiagnosticCode::TopologyLoopDiscontinuous;
                if (diagnostic.code == DiagnosticCode::TopologyLoopDiscontinuous)
                {
                    diagnosticContextComplete =
                        diagnostic.context.contains(QStringLiteral("maximumJoinGap"))
                        && diagnostic.context.contains
                            (QStringLiteral("numericalJoinEpsilon"))
                        && diagnostic.context.contains(QStringLiteral("entityId"))
                        && diagnostic.context.contains(QStringLiteral("sourceIndex"))
                        && diagnostic.context.contains(QStringLiteral("usedEntityIds"))
                        && diagnostic.context.contains(QStringLiteral("nodeCount"))
                        && diagnostic.context.contains(QStringLiteral("edgeCount"))
                        && diagnostic.context.contains
                            (QStringLiteral("connectionTolerance"));
                }
            }
            check(loop.status == OperationStatus::Failed && loop.value.has_value()
                && !loop.value->connectedLoop
                && std::abs(loop.value->maximumJoinGap - gap) <= 1.0e-6
                && hasDiscontinuousDiagnostic && diagnosticContextComplete,
                "measurable engineering gap is rejected");
        }

        const OperationResult<PathTopology> endpointToMiddle = build
        ({
            record(0U, 500U, { { 0, 0, 0 }, { 0, 10, 0 } }),
            record(1U, 501U, { { 0, 5, 0.01 }, { 0, 5, 5 }, { 0, 0, 0 } })
        });
        const OperationResult<TopologyLoopResult> endpointToMiddleLoop =
            endpointToMiddle.value->extractBestLoop({});
        check(endpointToMiddleLoop.status == OperationStatus::Failed
            && endpointToMiddleLoop.value.has_value()
            && endpointToMiddleLoop.value->maximumJoinGap >= 0.009,
            "endpoint near path middle does not create a strict loop");

        const OperationResult<PathTopology> nearIntersection = build
        ({
            record(0U, 510U, { { 0, 0, 0 }, { 0, 10, 0 } }),
            record(1U, 511U, { { 0.005, 5, -5 }, { 0.005, 5, 5 } })
        });
        const OperationResult<TopologyLoopResult> nearIntersectionLoop =
            nearIntersection.value->extractBestLoop({});
        check(nearIntersection.value->directlyConnected(510U, 511U)
            && !nearIntersectionLoop.succeeded(),
            "near segment intersection remains only candidate adjacency");

        const OperationResult<PathTopology> inclinedGap = build
        ({
            record(0U, 520U, { { 0, 0, 0.1 }, { 2, 10, 0 } }),
            record(1U, 521U, { { 2, 10, 0 }, { 1, 10, 10 } }),
            record(2U, 522U, { { 1, 10, 10 }, { -1, 0, 10 } }),
            record(3U, 523U, { { -1, 0, 10 }, { 0, 0, 0 } })
        });
        const OperationResult<TopologyLoopResult> inclinedGapLoop =
            inclinedGap.value->extractBestLoop({});
        check(inclinedGapLoop.status == OperationStatus::Failed
            && inclinedGapLoop.value.has_value()
            && inclinedGapLoop.value->maximumJoinGap >= 0.099,
            "inclined candidate with physical gap is rejected");

        std::vector<TopologyPathRecord> mixedCandidates = rectangle(600U, 0.0);
        mixedCandidates.front().points.front().z = 0.5;
        std::vector<TopologyPathRecord> strictSmall = rectangle(610U, 20.0);
        for (TopologyPathRecord& strictRecord : strictSmall)
        {
            strictRecord.sourceIndex += 4U;
        }
        mixedCandidates.insert
            (mixedCandidates.end(), strictSmall.begin(), strictSmall.end());
        mixedCandidates.push_back
            (record(8U, 620U, { { 0, 10, 0 }, { 20, 10, 0 } }));
        const OperationResult<PathTopology> mixedTopology = build(mixedCandidates);
        const OperationResult<TopologyLoopResult> mixedLoop =
            mixedTopology.value->extractBestLoop({});
        check(mixedLoop.succeeded() && mixedLoop.value.has_value()
            && mixedLoop.value->connectedLoop
            && mixedLoop.value->usedEntityIds
                == std::vector<EntityId>({ 610U, 611U, 612U, 613U }),
            "strict smaller loop wins before discontinuous larger loop scoring");

        const OperationResult<PathTopology> repeatedTopology = build(std::move(mixedCandidates));
        const OperationResult<TopologyLoopResult> repeatedLoop =
            repeatedTopology.value->extractBestLoop({});
        check(repeatedLoop.value.has_value()
            && mixedLoop.value.has_value()
            && repeatedLoop.value->usedEntityIds == mixedLoop.value->usedEntityIds
            && orderedPathDigest(repeatedLoop.value->orderedPath)
                == orderedPathDigest(mixedLoop.value->orderedPath),
            "strict loop selection is deterministic");
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

    QVector<CadItem*> appendDocumentSection(CadDocument& document, double joinGap)
    {
        return
        {
            document.appendEntity(makeDocumentLine(0, 0, joinGap, 0, 10, 0)),
            document.appendEntity(makeDocumentLine(0, 10, 0, 0, 10, 10)),
            document.appendEntity(makeDocumentLine(0, 10, 10, 0, 0, 10)),
            document.appendEntity(makeDocumentLine(0, 0, 10, 0, 0, 0))
        };
    }

    void testSectionAnalyzersRequireStrictLoop()
    {
        CadDocument exactDocument;
        const QVector<CadItem*> exactItems = appendDocumentSection(exactDocument, 0.0);
        const RotaryTubeSectionModel exactSection =
            RotaryTubeGeometryAnalyzer::buildSectionModel
                ({ exactItems.front() }, exactItems, 1.0);
        check(exactSection.valid, "strict tube section remains valid");

        CadDocument gapDocument;
        const QVector<CadItem*> gapItems = appendDocumentSection(gapDocument, 0.1);
        const RotaryTubeSectionModel gapSection =
            RotaryTubeGeometryAnalyzer::buildSectionModel
                ({ gapItems.front() }, gapItems, 1.0);
        check(!gapSection.valid && gapSection.errorMessage.contains(QStringLiteral("0.1")),
            "tube section rejects and reports physical join gap");

        const RotaryCutBoundaryAnalysis exactBoundary =
            RotaryCutBoundaryAnalyzer::analyze(exactItems, exactItems, exactSection, 1.0);
        check(exactBoundary.connectedLoop && exactBoundary.valid
            && exactBoundary.result == TubeCutResult::CutsLeftAndRight,
            "strict machining boundary reaches and passes production analysis");
        const RotaryCutBoundaryAnalysis gapBoundary =
            RotaryCutBoundaryAnalyzer::analyze(gapItems, gapItems, exactSection, 1.0);
        check(!gapBoundary.connectedLoop && !gapBoundary.valid
            && gapBoundary.maximumJoinGap >= 0.099
            && gapBoundary.errorMessage.contains(QStringLiteral("0.1")),
            "machining boundary stops before analysis when join gap exists");
    }

    TubeSectionGeometry standardTubeSection()
    {
        TubeSectionGeometry section;
        section.boundary =
        {
            { -5.0, 5.0 },
            { 5.0, 5.0 },
            { 5.0, -5.0 },
            { -5.0, -5.0 }
        };
        section.centerY = 0.0;
        section.centerZ = 0.0;
        section.yLength = 10.0;
        section.zWidth = 10.0;
        return section;
    }

    OperationResult<TubeCutAnalysis> classifyTubeCut
    (
        std::initializer_list<Vector3d> points,
        const TubeSectionGeometry& section = standardTubeSection()
    )
    {
        return TubeCutBoundaryClassifier::analyze
        (
            std::vector<Vector3d>(points),
            { 1U },
            0.0,
            section,
            createOperationContext(QStringLiteral("tube-cut-core-test"))
        );
    }

    bool hasDiagnostic
    (
        const OperationResult<TubeCutAnalysis>& result,
        DiagnosticCode code
    )
    {
        return std::any_of(result.diagnostics.cbegin(), result.diagnostics.cend(), [code](const Diagnostic& diagnostic)
        {
            return diagnostic.code == code;
        });
    }

    void testTubeCutBoundaryCore()
    {
        const OperationResult<TubeCutAnalysis> positive = classifyTubeCut
        ({
            { 0.0, -5.0, 5.0 },
            { 0.0, 5.0, 5.0 },
            { 0.0, 5.0, -5.0 },
            { 0.0, -5.0, -5.0 }
        });
        check(positive.succeeded() && positive.value.has_value()
            && positive.value->result == TubeCutResult::CutsLeftAndRight
            && positive.value->winding == 1
            && std::all_of
            (
                positive.value->seamResults.cbegin(),
                positive.value->seamResults.cend(),
                [](const SeamWindingResult& seam)
                {
                    return seam.usable && seam.winding == 1;
                }
            ),
            "tube cut positive winding separates both sides");

        const OperationResult<TubeCutAnalysis> negative = classifyTubeCut
        ({
            { 0.0, -5.0, 5.0 },
            { 0.0, -5.0, -5.0 },
            { 0.0, 5.0, -5.0 },
            { 0.0, 5.0, 5.0 }
        });
        check(negative.succeeded() && negative.value.has_value()
            && negative.value->result == TubeCutResult::CutsLeftAndRight
            && negative.value->winding == -1
            && std::all_of
            (
                negative.value->seamResults.cbegin(),
                negative.value->seamResults.cend(),
                [](const SeamWindingResult& seam)
                {
                    return seam.usable && seam.winding == -1;
                }
            ),
            "tube cut reverse winding separates both sides");

        const OperationResult<TubeCutAnalysis> bridge = classifyTubeCut
        ({
            { 0.0, -5.0, 5.0 },
            { 0.0, 5.0, 5.0 },
            { 0.0, 5.0, -5.0 },
            { 0.0, -5.0, -5.0 },
            { 1.0, -5.0, 5.0 },
            { 1.0, -5.0, -5.0 },
            { 1.0, 5.0, -5.0 },
            { 1.0, 5.0, 5.0 }
        });
        check(bridge.succeeded() && bridge.value.has_value()
            && bridge.value->result == TubeCutResult::KeepsLeftAndRight
            && bridge.value->winding == 0
            && std::all_of
            (
                bridge.value->seamResults.cbegin(),
                bridge.value->seamResults.cend(),
                [](const SeamWindingResult& seam)
                {
                    return seam.usable && seam.winding == 0;
                }
            )
            && hasDiagnostic(bridge, DiagnosticCode::CutBoundaryKeepsTubeConnected),
            "zero winding reports a remaining material bridge");

        const OperationResult<TubeCutAnalysis> seamTouch = classifyTubeCut
        ({
            { 0.0, -5.0, 5.0 },
            { 0.0, 0.0, 5.0 },
            { 0.0, 5.0, 5.0 },
            { 0.0, 0.0, 5.0 },
            { 0.0, 5.0, 5.0 },
            { 0.0, 5.0, -5.0 },
            { 0.0, -5.0, -5.0 }
        });
        check(seamTouch.succeeded() && seamTouch.value.has_value()
            && seamTouch.value->winding == 1
            && seamTouch.value->seamResults[0].positiveCrossingCount == 1
            && seamTouch.value->seamResults[0].touchCount == 1,
            "seam touch and return is not counted as crossing");

        const OperationResult<TubeCutAnalysis> seamOverlap = classifyTubeCut
        ({
            { 0.0, -5.0, 5.0 },
            { 0.0, 5.0, 5.0 },
            { 1.0, 5.0, 5.0 },
            { 1.0, 5.0, -5.0 },
            { 1.0, -5.0, -5.0 }
        });
        check(seamOverlap.succeeded() && seamOverlap.value.has_value()
            && seamOverlap.value->winding == 1
            && seamOverlap.value->seamResults[0].positiveCrossingCount == 1
            && seamOverlap.value->seamResults[0].overlapRunCount == 1,
            "seam overlap and cross is counted once");

        const OperationResult<TubeCutAnalysis> diagonal = classifyTubeCut
        ({
            { 0.0, -5.0, 5.0 },
            { 0.0, 5.0, -5.0 },
            { 0.0, 5.0, 5.0 },
            { 0.0, -5.0, -5.0 }
        });
        check(!diagonal.succeeded() && diagonal.value.has_value()
            && diagonal.value->result == TubeCutResult::Indeterminate
            && hasDiagnostic(diagonal, DiagnosticCode::CutBoundaryProjectionMismatch),
            "matching bounds with an interior diagonal is rejected");

        TubeSectionGeometry tinySection;
        tinySection.boundary =
        {
            { -0.000075, 0.000075 },
            { 0.000075, 0.000075 },
            { 0.000075, -0.000075 },
            { -0.000075, -0.000075 }
        };
        tinySection.yLength = 0.00015;
        tinySection.zWidth = 0.00015;
        const OperationResult<TubeCutAnalysis> degenerateSeams = classifyTubeCut
        ({
            { 0.0, -0.000075, 0.000075 },
            { 0.0, 0.000075, 0.000075 },
            { 0.0, 0.000075, -0.000075 },
            { 0.0, -0.000075, -0.000075 }
        }, tinySection);
        check(!degenerateSeams.succeeded() && degenerateSeams.value.has_value()
            && degenerateSeams.value->result == TubeCutResult::Indeterminate
            && hasDiagnostic(degenerateSeams, DiagnosticCode::CutBoundarySeamDegenerate),
            "unusable seam results remain indeterminate");

        std::array<SeamWindingResult, 4> disagreeingSeams;

        for (SeamWindingResult& seam : disagreeingSeams)
        {
            seam.usable = true;
            seam.winding = 1;
        }

        disagreeingSeams[2].winding = 0;
        check(TubeCutBoundaryClassifier::classifyWinding(1, disagreeingSeams)
            == TubeCutResult::Indeterminate,
            "disagreeing seam winding results remain indeterminate");

        const OperationResult<TubeCutAnalysis> multipleWinding = classifyTubeCut
        ({
            { 0.0, -5.0, 5.0 },
            { 0.0, 5.0, 5.0 },
            { 0.0, 5.0, -5.0 },
            { 0.0, -5.0, -5.0 },
            { 1.0, -5.0, 5.0 },
            { 1.0, 5.0, 5.0 },
            { 1.0, 5.0, -5.0 },
            { 1.0, -5.0, -5.0 }
        });
        check(!multipleWinding.succeeded() && multipleWinding.value.has_value()
            && multipleWinding.value->winding == 2
            && hasDiagnostic(multipleWinding, DiagnosticCode::CutBoundaryMultipleWinding),
            "multiple winding remains indeterminate");
    }

    struct TubeSectionFixture
    {
        TopologyInput input;
        PathTopology topology;
    };

    std::vector<TopologyPathRecord> rectangleRecords
    (
        std::size_t sourceIndex,
        EntityId entityId,
        double x,
        double minimumY,
        double maximumY,
        double minimumZ,
        double maximumZ
    )
    {
        return
        {
            record(sourceIndex, entityId,
                {{ x, minimumY, maximumZ }, { x, maximumY, maximumZ }}),
            record(sourceIndex + 1, entityId + 1,
                {{ x, maximumY, maximumZ }, { x, maximumY, minimumZ }}),
            record(sourceIndex + 2, entityId + 2,
                {{ x, maximumY, minimumZ }, { x, minimumY, minimumZ }}),
            record(sourceIndex + 3, entityId + 3,
                {{ x, minimumY, minimumZ }, { x, minimumY, maximumZ }})
        };
    }

    std::optional<TubeSectionFixture> tubeSectionFixture
    (
        std::vector<TopologyPathRecord> records
    )
    {
        TopologyInput input;
        input.contentRevision = 42U;
        input.records = std::move(records);
        const OperationResult<PathTopology> built = PathTopologyBuilder{}.build
        (
            input,
            PathTopologyTolerance{},
            task(QStringLiteral("tube-section-fixture"))
        );
        check(built.succeeded() && built.value.has_value(),
            "tube section fixture topology builds");
        if (!built.succeeded() || !built.value.has_value()) return std::nullopt;
        return TubeSectionFixture{ std::move(input), *built.value };
    }

    void testTubeSectionCore()
    {
        TubeSectionPolicy policy;
        const OperationContext context = createOperationContext
            (QStringLiteral("tube-section-core-test"));
        const auto exactFixture = tubeSectionFixture
            (rectangleRecords(0U, 100U, 0.0, -5.0, 5.0, -4.0, 4.0));
        if (!exactFixture.has_value()) return;
        const OperationResult<CoreTubeSectionModel> exact =
            TubeSectionAnalyzer::buildFromSelection
        (
            exactFixture->input,
            exactFixture->topology,
            { 100U },
            policy,
            context
        );
        check(exact.succeeded() && exact.value.has_value()
            && exact.value->outerBoundaryEntityIds.size() == 4U
            && std::abs(exact.value->geometry.yLength - 10.0) <= 1.0e-9
            && std::abs(exact.value->geometry.zWidth - 8.0) <= 1.0e-9,
            "exact rectangular tube section is recognized");
        if (!exact.value.has_value()) return;

        std::vector<TopologyPathRecord> nested = rectangleRecords
            (0U, 200U, 0.0, -10.0, 10.0, -8.0, 8.0);
        std::vector<TopologyPathRecord> inner = rectangleRecords
            (4U, 204U, 0.0, -3.0, 3.0, -2.0, 2.0);
        nested.insert(nested.end(), inner.begin(), inner.end());
        const auto nestedFixture = tubeSectionFixture(std::move(nested));
        if (!nestedFixture.has_value()) return;
        const OperationResult<CoreTubeSectionModel> best = TubeSectionAnalyzer::findBest
            (nestedFixture->input, nestedFixture->topology, policy, context);
        check(best.succeeded() && best.value.has_value()
            && best.value->outerBoundaryEntityIds.size() == 4U
            && std::find(best.value->outerBoundaryEntityIds.cbegin(),
                best.value->outerBoundaryEntityIds.cend(), 200U)
                != best.value->outerBoundaryEntityIds.cend()
            && std::find(best.value->outerBoundaryEntityIds.cbegin(),
                best.value->outerBoundaryEntityIds.cend(), 204U)
                == best.value->outerBoundaryEntityIds.cend(),
            "outer rectangle is selected over contained rectangle");
        if (!best.value.has_value()) return;

        std::vector<TopologyPathRecord> open = rectangleRecords
            (0U, 300U, 0.0, -5.0, 5.0, -5.0, 5.0);
        open.pop_back();
        const auto openFixture = tubeSectionFixture(std::move(open));
        if (!openFixture.has_value()) return;
        const OperationResult<CoreTubeSectionModel> openResult =
            TubeSectionAnalyzer::findBest
                (openFixture->input, openFixture->topology, policy, context);
        check(!openResult.succeeded() && !openResult.value.has_value(),
            "open rectangle is not recognized as tube section");

        std::vector<TopologyPathRecord> inclined
        {
            record(0U, 400U, {{ 0.0, -5.0, 5.0 }, { 0.4, 5.0, 5.0 }}),
            record(1U, 401U, {{ 0.4, 5.0, 5.0 }, { 0.4, 5.0, -5.0 }}),
            record(2U, 402U, {{ 0.4, 5.0, -5.0 }, { 0.0, -5.0, -5.0 }}),
            record(3U, 403U, {{ 0.0, -5.0, -5.0 }, { 0.0, -5.0, 5.0 }})
        };
        const auto inclinedFixture = tubeSectionFixture(std::move(inclined));
        if (!inclinedFixture.has_value()) return;
        const OperationResult<CoreTubeSectionModel> inclinedResult =
            TubeSectionAnalyzer::buildFromSelection
                (inclinedFixture->input, inclinedFixture->topology,
                 { 400U }, policy, context);
        check(!inclinedResult.succeeded()
            && std::any_of(inclinedResult.diagnostics.cbegin(), inclinedResult.diagnostics.cend(), [](const Diagnostic& value)
            {
                return value.code == DiagnosticCode::TubeSectionNotPerpendicular;
            }),
            "inclined section is rejected");

        std::vector<TopologyPathRecord> withInternalLine = rectangleRecords
            (0U, 500U, 0.0, -5.0, 5.0, -5.0, 5.0);
        withInternalLine.push_back(record(4U, 504U,
            {{ 0.0, -2.0, 0.0 }, { 0.0, 2.0, 0.0 }}));
        const auto lineFixture = tubeSectionFixture(std::move(withInternalLine));
        if (!lineFixture.has_value()) return;
        const OperationResult<CoreTubeSectionModel> lineSection =
            TubeSectionAnalyzer::buildFromSelection
                (lineFixture->input, lineFixture->topology, { 500U }, policy, context);
        check(lineSection.succeeded() && lineSection.value.has_value(),
            "tube section with internal line remains recognizable");
        if (!lineSection.value.has_value()) return;
        const auto lineClassification = TubeSectionAnalyzer::classifyInternalPaths
            (lineFixture->input, lineFixture->topology, *lineSection.value, policy, context);
        check(lineClassification.succeeded() && lineClassification.value.has_value()
            && std::find(lineClassification.value->physicalInteriorEntityIds.cbegin(),
                lineClassification.value->physicalInteriorEntityIds.cend(), 504U)
                != lineClassification.value->physicalInteriorEntityIds.cend(),
            "interior line is classified as physical interior");

        const auto nestedClassification = TubeSectionAnalyzer::classifyInternalPaths
            (nestedFixture->input, nestedFixture->topology, *best.value, policy, context);
        check(nestedClassification.succeeded() && nestedClassification.value.has_value()
            && std::find(nestedClassification.value->topologicalInteriorEntityIds.cbegin(),
                nestedClassification.value->topologicalInteriorEntityIds.cend(), 204U)
                != nestedClassification.value->topologicalInteriorEntityIds.cend(),
            "contained closed loop is classified as topological interior");
        check(nestedClassification.value.has_value()
            && std::none_of(best.value->outerBoundaryEntityIds.cbegin(),
                best.value->outerBoundaryEntityIds.cend(), [&nestedClassification](EntityId entityId)
                {
                    return std::find
                    (
                        nestedClassification.value->physicalInteriorEntityIds.cbegin(),
                        nestedClassification.value->physicalInteriorEntityIds.cend(),
                        entityId
                    ) != nestedClassification.value->physicalInteriorEntityIds.cend()
                    || std::find
                    (
                        nestedClassification.value->topologicalInteriorEntityIds.cbegin(),
                        nestedClassification.value->topologicalInteriorEntityIds.cend(),
                        entityId
                    ) != nestedClassification.value->topologicalInteriorEntityIds.cend();
                }),
            "outer boundary entities are never classified as interior");

        const auto analyzeCut = [&exact, &context](std::vector<Vector3d> path)
        {
            return TubeCutBoundaryClassifier::analyze
            (
                path,
                exact.value->outerBoundaryEntityIds,
                0.0,
                exact.value->geometry,
                context
            );
        };
        const auto positiveCut = analyzeCut
        ({
            { 0.0, -5.0, 4.0 }, { 0.0, 5.0, 4.0 },
            { 0.0, 5.0, -4.0 }, { 0.0, -5.0, -4.0 }
        });
        const auto negativeCut = analyzeCut
        ({
            { 0.0, -5.0, 4.0 }, { 0.0, -5.0, -4.0 },
            { 0.0, 5.0, -4.0 }, { 0.0, 5.0, 4.0 }
        });
        const auto bridgeCut = analyzeCut
        ({
            { 0.0, -5.0, 4.0 }, { 0.0, 5.0, 4.0 },
            { 0.0, 5.0, -4.0 }, { 0.0, -5.0, -4.0 },
            { 1.0, -5.0, 4.0 }, { 1.0, -5.0, -4.0 },
            { 1.0, 5.0, -4.0 }, { 1.0, 5.0, 4.0 }
        });
        check(positiveCut.value.has_value() && positiveCut.value->winding == 1
            && negativeCut.value.has_value() && negativeCut.value->winding == -1
            && bridgeCut.value.has_value()
            && bridgeCut.value->result == TubeCutResult::KeepsLeftAndRight,
            "recognized section geometry feeds positive negative and zero winding cuts");
    }

    struct PlanningFixture
    {
        ProcessPlanningInput input;
        PathTopology topology;
    };

    CoreTubeSectionModel planningTubeSection()
    {
        CoreTubeSectionModel model;
        model.contentRevision = 42U;
        model.geometry.boundary =
        {
            { -5.0, 4.0 }, { 5.0, 4.0 }, { 5.0, -4.0 }, { -5.0, -4.0 }
        };
        model.geometry.centerY = 0.0;
        model.geometry.centerZ = 0.0;
        model.geometry.yLength = 10.0;
        model.geometry.zWidth = 8.0;
        const auto prepared = TubeCutBoundaryClassifier::prepareSection
        (
            model.geometry,
            createOperationContext(QStringLiteral("planning-section"))
        );
        check(prepared.succeeded() && prepared.value.has_value(),
            "planning section prepares");
        if (prepared.value.has_value()) model.geometry = *prepared.value;
        return model;
    }

    std::optional<PlanningFixture> planningFixture
    (
        std::vector<TopologyPathRecord> records,
        const std::map<EntityId, std::pair<BoundaryRole, int>>& roles = {}
    )
    {
        TopologyInput topologyInput;
        topologyInput.contentRevision = 42U;
        topologyInput.records = records;
        const auto built = PathTopologyBuilder{}.build
        (
            topologyInput,
            PathTopologyTolerance{},
            task(QStringLiteral("process-planning-fixture"))
        );
        check(built.succeeded() && built.value.has_value(),
            "process planning topology builds");
        if (!built.value.has_value()) return std::nullopt;

        PlanningFixture fixture;
        fixture.topology = *built.value;
        fixture.input.contentRevision = topologyInput.contentRevision;
        fixture.input.topologyInput = std::move(topologyInput);
        fixture.input.topology = &fixture.topology;
        fixture.input.tubeSection = planningTubeSection();
        for (const TopologyPathRecord& topologyRecord : fixture.input.topologyInput.records)
        {
            PlanningEntity entity;
            entity.entityId = topologyRecord.entityId;
            entity.sourceIndex = topologyRecord.sourceIndex;
            entity.sourceKind = topologyRecord.sourceKind;
            entity.path.sourceEntityId = topologyRecord.entityId;
            entity.path.sourceKind = topologyRecord.sourceKind;
            entity.path.closed = topologyRecord.semanticallyClosed;
            for (std::size_t index = 0; index < topologyRecord.points.size(); ++index)
                entity.path.vertices.push_back
                    ({ topologyRecord.points[index], static_cast<double>(index) });
            const auto role = roles.find(topologyRecord.entityId);
            if (role != roles.end())
            {
                entity.boundaryRole = role->second.first;
                entity.boundaryPairId = role->second.second;
            }
            fixture.input.entities.push_back(std::move(entity));
        }
        return fixture;
    }

    int orderOf(const ProcessPlan& plan, EntityId entityId)
    {
        const auto found = std::find_if
        (
            plan.assignments.cbegin(), plan.assignments.cend(),
            [entityId](const ProcessAssignment& value) { return value.entityId == entityId; }
        );
        return found != plan.assignments.cend() ? found->processOrder : -1;
    }

    void testProcessPlanningCore()
    {
        const OperationContext context = createOperationContext
            (QStringLiteral("process-planning-core-test"));
        ProcessPlanningPolicy policy;
        policy.initialPosition = { 0.0, 0.0, 4.0 };

        auto nearestFixture = planningFixture
        ({
            record(0U, 1000U, {{ 2.0, -1.0, 4.0 }, { 3.0, -1.0, 4.0 }}),
            record(1U, 1001U, {{ 10.0, 1.0, 4.0 }, { 11.0, 1.0, 4.0 }})
        });
        if (!nearestFixture.has_value()) return;
        nearestFixture->input.topology = &nearestFixture->topology;
        const auto nearest = ProcessPlanBuilder::build
            (nearestFixture->input, policy, context);
        check(nearest.succeeded() && nearest.value.has_value()
            && orderOf(*nearest.value, 1000U) < orderOf(*nearest.value, 1001U),
            "nearest strategy chooses spatially nearest group");

        std::vector<TopologyPathRecord> breakRecords
        {
            record(0U, 1100U, {{ 1.0, -2.0, 4.0 }, { 2.0, -2.0, 4.0 }}),
            record(1U, 1101U, {{ 7.0, -2.0, 4.0 }, { 8.0, -2.0, 4.0 }})
        };
        std::vector<TopologyPathRecord> boundary = rectangleRecords
            (2U, 1110U, 5.0, -5.0, 5.0, -4.0, 4.0);
        breakRecords.insert(breakRecords.end(), boundary.begin(), boundary.end());
        std::map<EntityId, std::pair<BoundaryRole, int>> roles;
        for (EntityId id = 1110U; id < 1114U; ++id) roles[id] = { BoundaryRole::Break, 3 };
        auto breakFixture = planningFixture(std::move(breakRecords), roles);
        if (!breakFixture.has_value()) return;
        breakFixture->input.topology = &breakFixture->topology;
        ProcessPlanningPolicy constrainedNearestPolicy = policy;
        constrainedNearestPolicy.initialPosition = { 8.0, -2.0, 4.0 };
        const auto constrained = ProcessPlanBuilder::build
            (breakFixture->input, constrainedNearestPolicy, context);
        check(constrained.succeeded() && constrained.value.has_value()
            && orderOf(*constrained.value, 1100U) >= 0
            && orderOf(*constrained.value, 1101U) < orderOf(*constrained.value, 1100U)
            && orderOf(*constrained.value, 1100U) < orderOf(*constrained.value, 1110U),
            "nearest mode may choose a right group early but keeps Left before Break");

        auto lazyFixture = planningFixture
        ({
            record(0U, 1200U, {{ 1.0, -1.0, -4.0 }, { 2.0, -1.0, -4.0 }}),
            record(1U, 1201U, {{ 10.0, 1.0, 4.0 }, { 11.0, 1.0, 4.0 }})
        });
        if (!lazyFixture.has_value()) return;
        lazyFixture->input.topology = &lazyFixture->topology;
        policy.orderingStrategy = ProcessOrderingStrategy::LazyRotation;
        const auto lazy = ProcessPlanBuilder::build(lazyFixture->input, policy, context);
        check(lazy.succeeded() && lazy.value.has_value()
            && orderOf(*lazy.value, 1201U) < orderOf(*lazy.value, 1200U),
            "lazy strategy prioritizes lower rotation cost");

        breakFixture->input.topology = &breakFixture->topology;
        const auto constrainedLazy = ProcessPlanBuilder::build
            (breakFixture->input, policy, context);
        check(constrainedLazy.succeeded() && constrainedLazy.value.has_value()
            && orderOf(*constrainedLazy.value, 1100U) < orderOf(*constrainedLazy.value, 1110U),
            "break precedence blocks lazy strategy from selecting boundary early");

        std::vector<TopologyPathRecord> twoBreaks
        {
            record(0U, 1300U, {{ 1.0, -2.0, 4.0 }, { 2.0, -2.0, 4.0 }}),
            record(1U, 1301U, {{ 7.0, -2.0, 4.0 }, { 8.0, -2.0, 4.0 }})
        };
        auto firstBoundary = rectangleRecords(2U, 1310U, 5.0, -5.0, 5.0, -4.0, 4.0);
        auto secondBoundary = rectangleRecords(6U, 1320U, 10.0, -5.0, 5.0, -4.0, 4.0);
        twoBreaks.insert(twoBreaks.end(), firstBoundary.begin(), firstBoundary.end());
        twoBreaks.insert(twoBreaks.end(), secondBoundary.begin(), secondBoundary.end());
        std::map<EntityId, std::pair<BoundaryRole, int>> twoRoles;
        for (EntityId id = 1310U; id < 1314U; ++id) twoRoles[id] = { BoundaryRole::Break, 7 };
        for (EntityId id = 1320U; id < 1324U; ++id) twoRoles[id] = { BoundaryRole::Break, 2 };
        auto twoFixture = planningFixture(std::move(twoBreaks), twoRoles);
        if (!twoFixture.has_value()) return;
        twoFixture->input.topology = &twoFixture->topology;
        policy.orderingStrategy = ProcessOrderingStrategy::NearestNext;
        const auto twoPlan = ProcessPlanBuilder::build(twoFixture->input, policy, context);
        check(twoPlan.succeeded() && twoPlan.value.has_value()
            && orderOf(*twoPlan.value, 1300U) < orderOf(*twoPlan.value, 1310U)
            && orderOf(*twoPlan.value, 1301U) < orderOf(*twoPlan.value, 1320U)
            && orderOf(*twoPlan.value, 1310U) < orderOf(*twoPlan.value, 1320U),
            "multiple break boundaries independently enforce spatial precedence");

        std::vector<TopologyPathRecord> mixedRecords
        {
            record(0U, 1400U, {{ 3.0, 0.0, 4.0 }, { 7.0, 0.0, 4.0 }})
        };
        auto mixedBoundary = rectangleRecords(1U, 1410U, 5.0, -5.0, 5.0, -4.0, 4.0);
        mixedRecords.insert(mixedRecords.end(), mixedBoundary.begin(), mixedBoundary.end());
        std::map<EntityId, std::pair<BoundaryRole, int>> mixedRoles;
        for (EntityId id = 1410U; id < 1414U; ++id) mixedRoles[id] = { BoundaryRole::Break, 1 };
        auto mixedFixture = planningFixture(std::move(mixedRecords), mixedRoles);
        if (!mixedFixture.has_value()) return;
        mixedFixture->input.topology = &mixedFixture->topology;
        const auto mixed = ProcessPlanBuilder::build(mixedFixture->input, policy, context);
        check(!mixed.succeeded() && std::any_of
        (
            mixed.diagnostics.cbegin(), mixed.diagnostics.cend(),
            [](const Diagnostic& diagnostic)
            {
                return diagnostic.code == DiagnosticCode::ProcessPlanningBoundaryClassificationFailed;
            }
        ), "mixed boundary side rejects process plan");

        CadDocument document;
        CadItem* unchanged = document.appendEntity(makeDocumentLine(0, 0, 0, 1, 0, 0));
        unchanged->m_processOrder = 27;
        ProcessPlan stalePlan;
        stalePlan.contentRevision = document.contentRevision() + 1U;
        stalePlan.assignments.push_back({ unchanged->m_entityId, 0, -1, false, std::nullopt });
        const OperationReport apply = LegacyProcessPlanAdapter{}.apply(document, stalePlan, context);
        check(!apply.succeeded() && unchanged->m_processOrder == 27,
            "revision conflict leaves CadItem unchanged");

        ProcessPlan missingEntityPlan;
        missingEntityPlan.contentRevision = document.contentRevision();
        missingEntityPlan.assignments.push_back
            ({ unchanged->m_entityId, 0, -1, false, std::nullopt });
        missingEntityPlan.assignments.push_back
            ({ unchanged->m_entityId + 1000U, 1, -1, false, std::nullopt });
        const OperationReport missingEntityApply = LegacyProcessPlanAdapter{}.apply
            (document, missingEntityPlan, context);
        check(!missingEntityApply.succeeded() && unchanged->m_processOrder == 27,
            "missing EntityId leaves CadItem unchanged");

        CadDocument cycleDocument;
        CadItem* cycleFirst = cycleDocument.appendEntity
            (makeDocumentLine(0, 0, 0, 1, 0, 0));
        CadItem* cycleSecond = cycleDocument.appendEntity
            (makeDocumentLine(2, 0, 0, 3, 0, 0));
        cycleFirst->m_processOrder = 41;
        cycleSecond->m_processOrder = 42;
        ProcessPlan cyclePlan;
        cyclePlan.contentRevision = cycleDocument.contentRevision();
        cyclePlan.groups.push_back
            ({ 0, ProcessGroupKind::SingleEntity, false, { cycleFirst->m_entityId } });
        cyclePlan.groups.push_back
            ({ 1, ProcessGroupKind::SingleEntity, false, { cycleSecond->m_entityId } });
        cyclePlan.assignments.push_back
            ({ cycleFirst->m_entityId, 0, -1, false, std::nullopt });
        cyclePlan.assignments.push_back
            ({ cycleSecond->m_entityId, 1, -1, false, std::nullopt });
        cyclePlan.precedenceConstraints.push_back({ 0, 1, 1 });
        cyclePlan.precedenceConstraints.push_back({ 1, 0, 2 });
        const OperationReport cycleApply = LegacyProcessPlanAdapter{}.apply
            (cycleDocument, cyclePlan, context);
        check(!cycleApply.succeeded()
            && cycleFirst->m_processOrder == 41
            && cycleSecond->m_processOrder == 42,
            "precedence cycle is rejected without partial CadItem writes");

        CadDocument mixedDocument;
        mixedDocument.appendEntity(makeDocumentLine(0, 0, 4, 1, 0, 4));
        auto pointEntity = std::make_unique<DRW_Point>();
        pointEntity->basePoint = { 0.0, 0.0, 0.0 };
        CadItem* pointItem = mixedDocument.appendEntity(std::move(pointEntity));
        cadcam::topology::PathTopology mixedTopology;
        CoreTubeSectionModel mixedSection = planningTubeSection();
        mixedSection.contentRevision = mixedDocument.contentRevision();
        const auto captured = LegacyProcessPlanAdapter{}.capture
        (
            mixedDocument, mixedSection, policy.connectionTolerance,
            mixedTopology, context
        );
        check(captured.succeeded() && captured.value.has_value()
            && captured.value->entities.size() == 2U
            && captured.value->topologyInput.records.size() == 1U,
            "unsupported entities remain in planning input but not topology");
        if (captured.value.has_value())
        {
            const auto mixedDocumentPlan = ProcessPlanBuilder::build
                (*captured.value, policy, context);
            check(mixedDocumentPlan.succeeded() && mixedDocumentPlan.value.has_value()
                && std::any_of
                (
                    mixedDocumentPlan.value->exclusions.cbegin(),
                    mixedDocumentPlan.value->exclusions.cend(),
                    [pointItem](const ProcessExclusion& exclusion)
                    {
                        return exclusion.entityId == pointItem->m_entityId
                            && exclusion.reason == ProcessExclusionReason::UnsupportedGeometry;
                    }
                ), "unsupported entity becomes a ProcessPlan exclusion");
        }
    }

    void testPlanarProcessPlanningCore()
    {
        const OperationContext context = createOperationContext
            (QStringLiteral("planar-process-planning-core-test"));
        auto entity = []
        (
            EntityId id,
            std::size_t sourceIndex,
            SourceGeometryKind kind,
            std::initializer_list<Vector3d> points,
            bool closed = false,
            std::optional<double> startParameter = std::nullopt
        )
        {
            PlanarPlanningEntity value;
            value.entityId = id;
            value.sourceIndex = sourceIndex;
            value.sourceKind = kind;
            value.sourceEntity.id = id;
            value.sourceEntity.kind = kind;
            value.path.sourceEntityId = id;
            value.path.sourceKind = kind;
            value.path.closed = closed;
            std::size_t parameter = 0U;
            for (const Vector3d& point : points)
                value.path.vertices.push_back({ point, static_cast<double>(parameter++) });
            value.customStartParameter = startParameter;
            return value;
        };
        auto assignment = [](const ProcessPlan& plan, EntityId id)
            -> const ProcessAssignment*
        {
            const auto found = std::find_if(plan.assignments.cbegin(), plan.assignments.cend(),
                [id](const ProcessAssignment& value) { return value.entityId == id; });
            return found == plan.assignments.cend() ? nullptr : &*found;
        };

        PlanarProcessPlanningPolicy policy;
        policy.allowReverse = true;
        policy.preserveUserDirection = true;
        policy.hasInitialPosition = true;
        policy.initialPosition = { 0.0, 0.0, 0.0 };

        PlanarProcessPlanningInput nearestInput;
        nearestInput.contentRevision = 7U;
        nearestInput.entities =
        {
            entity(2000U, 0U, SourceGeometryKind::Line,
                { { 10.0, 0.0, 0.0 }, { 11.0, 0.0, 0.0 } }),
            entity(2001U, 1U, SourceGeometryKind::Line,
                { { 3.0, 0.0, 0.0 }, { 4.0, 0.0, 0.0 } }),
            entity(2002U, 2U, SourceGeometryKind::Line,
                { { 20.0, 0.0, 0.0 }, { 21.0, 0.0, 0.0 } })
        };
        const auto nearest = PlanarProcessPlanBuilder::build(nearestInput, policy, context);
        check(nearest.succeeded() && nearest.value.has_value()
            && nearest.value->mode == ProcessPlanMode::Planar3Axis
            && nearest.value->assignments.size() == 3U
            && nearest.value->assignments[0].entityId == 2001U
            && nearest.value->assignments[1].entityId == 2000U
            && nearest.value->assignments[2].entityId == 2002U,
            "planar plan orders three open paths by nearest entry");

        PlanarProcessPlanningInput reverseInput;
        reverseInput.contentRevision = 8U;
        reverseInput.entities =
        {
            entity(2100U, 0U, SourceGeometryKind::Line,
                { { 10.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0 } })
        };
        const auto reversed = PlanarProcessPlanBuilder::build(reverseInput, policy, context);
        check(reversed.succeeded() && reversed.value.has_value()
            && reversed.value->assignments.size() == 1U
            && reversed.value->assignments.front().reverse,
            "planar plan chooses the closer reverse entry");

        PlanarProcessPlanningInput stableInput;
        stableInput.contentRevision = 9U;
        stableInput.entities =
        {
            entity(2202U, 2U, SourceGeometryKind::Line,
                { { 5.0, 0.0, 0.0 }, { 6.0, 0.0, 0.0 } }),
            entity(2201U, 1U, SourceGeometryKind::Line,
                { { 0.0, 5.0, 0.0 }, { 0.0, 6.0, 0.0 } }),
            entity(2200U, 1U, SourceGeometryKind::Line,
                { { -5.0, 0.0, 0.0 }, { -6.0, 0.0, 0.0 } })
        };
        const auto stable = PlanarProcessPlanBuilder::build(stableInput, policy, context);
        check(stable.succeeded() && stable.value.has_value()
            && stable.value->assignments.front().entityId == 2200U,
            "planar equal distances use source index then entity id");

        PlanarProcessPlanningInput closedInput;
        closedInput.contentRevision = 10U;
        closedInput.entities =
        {
            entity(2300U, 0U, SourceGeometryKind::Circle,
                { { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 } }, true),
            entity(2301U, 1U, SourceGeometryKind::Ellipse,
                { { 2.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 } }, true),
            entity(2302U, 2U, SourceGeometryKind::Polyline,
                { { 3.0, 0.0, 0.0 }, { 4.0, 0.0, 0.0 }, { 4.0, 1.0, 0.0 } },
                true, 2.0)
        };
        const auto closed = PlanarProcessPlanBuilder::build(closedInput, policy, context);
        const ProcessAssignment* circle = closed.value.has_value()
            ? assignment(*closed.value, 2300U) : nullptr;
        const ProcessAssignment* ellipse = closed.value.has_value()
            ? assignment(*closed.value, 2301U) : nullptr;
        const ProcessAssignment* polyline = closed.value.has_value()
            ? assignment(*closed.value, 2302U) : nullptr;
        check(closed.succeeded() && circle != nullptr && ellipse != nullptr
            && circle->startParameter.has_value() && ellipse->startParameter.has_value()
            && std::abs(*circle->startParameter - 1.57079632679489661923) <= 1.0e-12
            && std::abs(*ellipse->startParameter - 1.57079632679489661923) <= 1.0e-12,
            "planar circle and ellipse retain north-pole starts");
        check(polyline != nullptr && polyline->startParameter == std::optional<double>(2.0),
            "planar closed polyline retains custom start");
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
        RotaryPathTopology circleTopology({ circleItem }, tolerance);
        const RotaryPathLoopResult circleLoop = circleTopology.extractBestLoop({ circleItem });
        check(circleLoop.valid && circleLoop.connectedLoop
            && circleLoop.maximumJoinGap == 0.0,
            "complete circle is semantically and strictly closed");

        CadDocument ellipseDocument;
        CadItem* ellipseItem = ellipseDocument.appendEntity(makeTopologyEllipse());
        RotaryPathTopology ellipseTopology({ ellipseItem }, tolerance);
        const RotaryPathLoopResult ellipseLoop =
            ellipseTopology.extractBestLoop({ ellipseItem });
        check(ellipseLoop.valid && ellipseLoop.connectedLoop
            && ellipseLoop.maximumJoinGap == 0.0,
            "complete ellipse is semantically and strictly closed");

        CadDocument polylineDocument;
        CadItem* polylineItem = polylineDocument.appendEntity(makeTopologyBulgePolyline());
        RotaryPathTopology polylineTopology({ polylineItem }, tolerance);
        const RotaryPathLoopResult polylineLoop =
            polylineTopology.extractBestLoop({ polylineItem });
        check(polylineLoop.valid && polylineLoop.connectedLoop
            && polylineLoop.maximumJoinGap == 0.0,
            "closed bulge polyline is semantically and strictly closed");

        CadDocument splineDocument;
        std::unique_ptr<DRW_Spline> closedSpline = makeTopologySpline();
        closedSpline->flags |= 1;
        CadItem* splineItem = splineDocument.appendEntity(std::move(closedSpline));
        RotaryPathTopology splineTopology({ splineItem }, tolerance);
        const RotaryPathLoopResult splineLoop =
            splineTopology.extractBestLoop({ splineItem });
        check(splineLoop.valid && splineLoop.connectedLoop
            && splineLoop.maximumJoinGap == 0.0,
            "closed spline is semantically and strictly closed");

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

    void testLegacyWrapperStrictClosureAndSourceBoundary()
    {
        CadDocument document;
        QVector<CadItem*> items;
        items.push_back(document.appendEntity(makeDocumentLine(0, 0, 0.5, 0, 10, 0)));
        items.push_back(document.appendEntity(makeDocumentLine(0, 10, 0, 0, 10, 10)));
        items.push_back(document.appendEntity(makeDocumentLine(0, 10, 10, 0, 0, 10)));
        items.push_back(document.appendEntity(makeDocumentLine(0, 0, 10, 0, 0, 0)));
        RotaryPathTopology topology(items, PathTopologyTolerance{});
        const RotaryPathLoopResult loop = topology.extractSeededLoop({ items[0] });
        bool hasDiscontinuousDiagnostic = false;
        for (const Diagnostic& diagnostic : topology.diagnostics())
        {
            hasDiscontinuousDiagnostic = hasDiscontinuousDiagnostic
                || diagnostic.code == DiagnosticCode::TopologyLoopDiscontinuous;
        }
        check(!loop.valid && loop.maximumJoinGap >= 0.499
            && hasDiscontinuousDiagnostic
            && loop.errorMessage.contains(QStringLiteral("0.5")),
            "legacy wrapper exposes strict discontinuity diagnostic");

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

        const QByteArray forbiddenSemantics[] =
        {
            QByteArray("approximately") + QByteArray("Closed"),
            QByteArray("TopologyApproximate") + QByteArray("Closure"),
            (QString::fromUtf8("几乎") + QString::fromUtf8("闭合")).toUtf8(),
            (QString::fromUtf8("近似") + QString::fromUtf8("闭合")).toUtf8()
        };
        for (const QString& root : { QStringLiteral("include"), QStringLiteral("src") })
        {
            QDirIterator iterator
            (
                root,
                { QStringLiteral("*.h"), QStringLiteral("*.cpp") },
                QDir::Files,
                QDirIterator::Subdirectories
            );
            while (iterator.hasNext())
            {
                QFile source(iterator.next());
                if (!source.open(QIODevice::ReadOnly))
                {
                    check(false, "strict closure source file opens");
                    continue;
                }
                const QByteArray text = source.readAll();
                for (const QByteArray& term : forbiddenSemantics)
                {
                    check(!text.contains(term), "legacy loose closure semantics removed");
                }
            }
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
    testStrictLoopContinuity();
    testAdapterCancellationRevisionAndDeterminism();
    testSectionAnalyzersRequireStrictLoop();
    testTubeCutBoundaryCore();
    testTubeSectionCore();
    testProcessPlanningCore();
    testPlanarProcessPlanningCore();
    testLegacyAdapterValidationAndOrdering();
    testLegacyWrapperPublicApi();
    testLegacyAdapterProcessSemanticsAndTypes();
    testLegacyWrapperStrictClosureAndSourceBoundary();
    testLegacyParity();
    return failures;
}
