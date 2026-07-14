#include "GeometrySnapshotTests.h"

#include "CadDocument.h"
#include "CadItem.h"
#include "application/geometry/DocumentGeometrySnapshotBuilder.h"
#include "application/geometry/GeometrySnapshotCompiler.h"
#include "application/process/DocumentProcessState.h"
#include "compatibility/legacy/GeometrySnapshotParityVerifier.h"

#include <QCryptographicHash>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <thread>

namespace
{
    int failures = 0;

    void check(bool condition, const char* name)
    {
        if (!condition)
        {
            ++failures;
            std::cerr << "FAILED: " << name << '\n';
        }
    }

    OperationContext context(const QString& operation)
    {
        return { QStringLiteral("geometry-snapshot-test"), operation };
    }

    std::unique_ptr<DRW_Line> makeLine(double offset = 0.0)
    {
        auto entity = std::make_unique<DRW_Line>();
        entity->basePoint = DRW_Coord(offset, 0.0, 0.0);
        entity->secPoint = DRW_Coord(offset + 10.0, 2.0, 0.0);
        entity->layer = "snapshot";
        entity->color = 1;
        return entity;
    }

    std::unique_ptr<DRW_Arc> makeArc()
    {
        auto entity = std::make_unique<DRW_Arc>();
        entity->basePoint = DRW_Coord(5.0, 5.0, 0.0);
        entity->radious = 3.0;
        entity->staangle = 0.2;
        entity->endangle = 2.4;
        entity->extPoint = DRW_Coord(0.0, 0.0, 1.0);
        return entity;
    }

    std::unique_ptr<DRW_Circle> makeCircle(double radius = 4.0)
    {
        auto entity = std::make_unique<DRW_Circle>();
        entity->basePoint = DRW_Coord(15.0, 0.0, 0.0);
        entity->radious = radius;
        entity->extPoint = DRW_Coord(0.0, 0.0, 1.0);
        return entity;
    }

    std::unique_ptr<DRW_Ellipse> makeEllipse()
    {
        auto entity = std::make_unique<DRW_Ellipse>();
        entity->basePoint = DRW_Coord(20.0, 0.0, 0.0);
        entity->secPoint = DRW_Coord(6.0, 0.0, 0.0);
        entity->ratio = 0.5;
        entity->staparam = 0.0;
        entity->endparam = 6.28318530717958647692;
        entity->extPoint = DRW_Coord(0.0, 0.0, 1.0);
        return entity;
    }

    std::unique_ptr<DRW_Polyline> makePolyline()
    {
        auto entity = std::make_unique<DRW_Polyline>();
        entity->flags = 1;
        entity->extPoint = DRW_Coord(0.0, 0.0, 1.0);
        const double points[4][2] =
        {
            { 0.0, 0.0 }, { 8.0, 0.0 }, { 8.0, 5.0 }, { 0.0, 5.0 }
        };
        for (const auto& point : points)
        {
            auto vertex = std::make_shared<DRW_Vertex>();
            vertex->basePoint = DRW_Coord(point[0], point[1], 0.0);
            entity->vertlist.push_back(vertex);
        }
        entity->vertexcount = static_cast<int>(entity->vertlist.size());
        return entity;
    }

    std::unique_ptr<DRW_LWPolyline> makeLWPolyline()
    {
        auto entity = std::make_unique<DRW_LWPolyline>();
        entity->flags = 0;
        entity->elevation = 2.0;
        entity->extPoint = DRW_Coord(0.0, 0.0, 1.0);
        for (int index = 0; index < 3; ++index)
        {
            auto vertex = std::make_shared<DRW_Vertex2D>();
            vertex->x = static_cast<double>(index) * 4.0;
            vertex->y = index == 1 ? 3.0 : 0.0;
            vertex->bulge = index == 0 ? 0.25 : 0.0;
            entity->vertlist.push_back(vertex);
        }
        entity->vertexnum = static_cast<int>(entity->vertlist.size());
        return entity;
    }

    std::unique_ptr<DRW_Spline> makeSpline()
    {
        auto entity = std::make_unique<DRW_Spline>();
        entity->degree = 3;
        entity->flags = 4;
        entity->knotslist = { 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0 };
        entity->weightlist = { 1.0, 0.8, 0.8, 1.0 };
        entity->controllist =
        {
            std::make_shared<DRW_Coord>(0.0, 0.0, 0.0),
            std::make_shared<DRW_Coord>(4.0, 8.0, 1.0),
            std::make_shared<DRW_Coord>(8.0, -4.0, 2.0),
            std::make_shared<DRW_Coord>(12.0, 0.0, 3.0)
        };
        entity->ncontrol = static_cast<dint32>(entity->controllist.size());
        entity->nknots = static_cast<dint32>(entity->knotslist.size());
        return entity;
    }

    void appendBytes(QByteArray& bytes, const void* value, std::size_t size)
    {
        bytes.append(static_cast<const char*>(value), static_cast<qsizetype>(size));
    }

    QByteArray snapshotDigest(const GeometrySnapshot& snapshot)
    {
        QByteArray bytes;
        appendBytes(bytes, &snapshot.contentRevision, sizeof(snapshot.contentRevision));
        for (const GeometrySnapshotEntry& entry : snapshot.entries)
        {
            appendBytes(bytes, &entry.sourceIndex, sizeof(entry.sourceIndex));
            appendBytes(bytes, &entry.attributes.entityId, sizeof(entry.attributes.entityId));
            const int kind = static_cast<int>(entry.sourceKind);
            const int status = static_cast<int>(entry.status);
            appendBytes(bytes, &kind, sizeof(kind));
            appendBytes(bytes, &status, sizeof(status));
            const bool hasPath = entry.path.has_value();
            appendBytes(bytes, &hasPath, sizeof(hasPath));
            if (!hasPath)
            {
                continue;
            }
            appendBytes(bytes, &entry.path->closed, sizeof(entry.path->closed));
            const std::size_t count = entry.path->vertices.size();
            appendBytes(bytes, &count, sizeof(count));
            for (const cadcam::geometry::PathVertex3D& vertex : entry.path->vertices)
            {
                appendBytes(bytes, &vertex.position.x, sizeof(double));
                appendBytes(bytes, &vertex.position.y, sizeof(double));
                appendBytes(bytes, &vertex.position.z, sizeof(double));
                appendBytes(bytes, &vertex.sourceParameter, sizeof(double));
            }
            for (const Diagnostic& diagnostic : entry.diagnostics)
            {
                const int code = static_cast<int>(diagnostic.code);
                appendBytes(bytes, &code, sizeof(code));
            }
        }
        return QCryptographicHash::hash(bytes, QCryptographicHash::Sha256);
    }

    GeometrySourceSnapshot captureMixedDocument(CadDocument& document)
    {
        document.appendEntity(makeLine());
        document.appendEntity(makeArc());
        document.appendEntity(makeCircle());
        document.appendEntity(makeEllipse());
        document.appendEntity(makePolyline());
        document.appendEntity(makeLWPolyline());
        document.appendEntity(makeSpline());
        DocumentGeometrySnapshotBuilder builder;
        const OperationResult<GeometrySourceSnapshot> captured = builder.capture
            (document, context(QStringLiteral("capture-mixed-document")));
        check(captured.succeeded() && captured.value.has_value(),
            "mixed source snapshot capture succeeds");
        check(captured.value.has_value() && captured.value->entries.size() == 7U,
            "mixed source snapshot retains every entity");
        return captured.value.value_or(GeometrySourceSnapshot{});
    }

    OperationResult<GeometrySnapshot> compileSnapshot
    (
        const GeometrySourceSnapshot& source,
        GeometryExecutionMode mode,
        const TaskContext& task = {}
    )
    {
        GeometrySnapshotCompiler compiler;
        TaskContext effectiveTask = task;
        if (effectiveTask.operationContext.operationName.isEmpty())
        {
            effectiveTask.operationContext = context(QStringLiteral("compile-snapshot"));
        }
        return compiler.compile
            (source, cadcam::geometry::SamplingPolicy{}, mode, effectiveTask);
    }

    void testCaptureCompileAndParity()
    {
        CadDocument document;
        const GeometrySourceSnapshot source = captureMixedDocument(document);
        const OperationResult<GeometrySnapshot> serial = compileSnapshot
            (source, GeometryExecutionMode::Serial);
        const OperationResult<GeometrySnapshot> parallel = compileSnapshot
            (source, GeometryExecutionMode::Parallel);
        const OperationResult<GeometrySnapshot> parallelAgain = compileSnapshot
            (source, GeometryExecutionMode::Parallel);
        GeometrySourceSnapshot shuffledSource = source;
        std::reverse(shuffledSource.entries.begin(), shuffledSource.entries.end());
        const OperationResult<GeometrySnapshot> shuffled = compileSnapshot
            (shuffledSource, GeometryExecutionMode::Parallel);
        check(serial.status == OperationStatus::Success && serial.value.has_value(),
            "serial mixed snapshot compiles");
        check(parallel.status == OperationStatus::Success && parallel.value.has_value(),
            "parallel mixed snapshot compiles");
        check(serial.value.has_value() && parallel.value.has_value()
            && snapshotDigest(*serial.value) == snapshotDigest(*parallel.value),
            "serial and parallel snapshot digests match");
        check(parallel.value.has_value() && parallelAgain.value.has_value()
            && snapshotDigest(*parallel.value) == snapshotDigest(*parallelAgain.value),
            "repeated parallel snapshot output is deterministic");
        check(serial.value.has_value() && shuffled.value.has_value()
            && snapshotDigest(*serial.value) == snapshotDigest(*shuffled.value),
            "parallel merge is ordered strictly by source index");
        if (serial.value.has_value())
        {
            GeometrySnapshotParityVerifier verifier;
            const OperationResult<GeometrySnapshotParityReport> parity = verifier.verify
                (document, *serial.value, context(QStringLiteral("verify-mixed-parity")));
            check(parity.status == OperationStatus::Success
                && parity.value.has_value() && parity.value->equivalent,
                "snapshot shadow parity succeeds");
            check(parity.value.has_value()
                && parity.value->maximumPointDistance <= 1.0e-6,
                "snapshot shadow parity maximum error");
        }
    }

    void testPartialFailureAndCancellation()
    {
        CadDocument invalidDocument;
        invalidDocument.appendEntity(makeLine());
        invalidDocument.appendEntity(makeCircle(0.0));
        DocumentGeometrySnapshotBuilder builder;
        const OperationResult<GeometrySourceSnapshot> captured = builder.capture
            (invalidDocument, context(QStringLiteral("capture-invalid-document")));
        check(captured.status == OperationStatus::PartialSuccess
            && captured.value.has_value() && captured.value->entries.size() == 2U,
            "invalid source remains in partial source snapshot");
        const OperationResult<GeometrySnapshot> compiled = compileSnapshot
            (*captured.value, GeometryExecutionMode::Parallel);
        check(compiled.status == OperationStatus::PartialSuccess
            && compiled.value.has_value() && compiled.value->entries.size() == 2U,
            "one invalid entity produces partial compiled snapshot");
        check(compiled.value.has_value()
            && !compiled.value->entries[1].path.has_value()
            && !compiled.value->entries[1].diagnostics.isEmpty(),
            "failed snapshot entry retains diagnostics");

        CadDocument cancellationDocument;
        std::vector<std::unique_ptr<DRW_Entity>> entities;
        for (int index = 0; index < 32; ++index)
        {
            entities.push_back(makeLine(static_cast<double>(index) * 20.0));
        }
        cancellationDocument.appendEntities(std::move(entities), false);
        const OperationResult<GeometrySourceSnapshot> cancellationSource = builder.capture
            (cancellationDocument, context(QStringLiteral("capture-cancellation-document")));
        CancellationSource cancellation;
        GeometryBuildProgress finalProgress;
        TaskContext task;
        task.operationContext = context(QStringLiteral("cancel-serial-snapshot"));
        task.cancellationToken = cancellation.token();
        task.progressCallback = [&cancellation, &finalProgress]
        (const GeometryBuildProgress& progress)
        {
            finalProgress = progress;
            if (progress.completedCount >= 3U)
            {
                cancellation.cancel();
            }
        };
        const OperationResult<GeometrySnapshot> cancelled = compileSnapshot
            (*cancellationSource.value, GeometryExecutionMode::Serial, task);
        check(cancelled.status == OperationStatus::Cancelled
            && cancelled.value.has_value() && cancelled.value->entries.size() == 3U,
            "cancellation retains completed entries and stops new work");
        check(finalProgress.completedCount == 3U && finalProgress.totalCount == 32U,
            "cancellation progress reports completed and total counts");
    }

    void testRevisionAndThreadBoundaries()
    {
        CadDocument document;
        const std::uint64_t initialRevision = document.contentRevision();
        std::vector<std::unique_ptr<DRW_Entity>> entities;
        entities.push_back(makeLine(0.0));
        entities.push_back(makeLine(20.0));
        entities.push_back(makeLine(40.0));
        check(document.appendEntities(std::move(entities), false) == 3
            && document.contentRevision() == initialRevision + 1U,
            "batch append advances revision once");

        CadItem* item = document.m_entities.front().get();
        const cadcam::geometry::EntityId entityId = item->m_entityId;
        const std::uint64_t beforeUndo = document.contentRevision();
        std::pair<std::unique_ptr<DRW_Entity>, std::unique_ptr<CadItem>> removed;
        {
            auto batch = document.beginContentChangeBatch();
            removed = document.takeEntity(item);
        }
        check(document.contentRevision() == beforeUndo + 1U,
            "undo removal advances revision once");
        const std::uint64_t beforeRedo = document.contentRevision();
        CadItem* restored = nullptr;
        {
            auto batch = document.beginContentChangeBatch();
            restored = document.appendEntity
                (std::move(removed.first), std::move(removed.second));
        }
        check(restored != nullptr && restored->m_entityId == entityId
            && document.contentRevision() == beforeRedo + 1U,
            "redo restoration preserves EntityId and advances revision once");

        const std::uint64_t beforeProcessState = document.contentRevision();
        cadcam::process::DocumentProcessState processState;
        processState.setDirection(restored->m_entityId,
            cadcam::process::DirectionPreference::Reverse);
        check(document.contentRevision() == beforeProcessState,
            "process direction state does not advance content revision");

        DocumentGeometrySnapshotBuilder builder;
        const OperationResult<GeometrySourceSnapshot> captured = builder.capture
            (document, context(QStringLiteral("capture-before-stale")));
        const OperationResult<GeometrySnapshot> compiled = compileSnapshot
            (*captured.value, GeometryExecutionMode::Serial);
        document.appendEntity(makeLine(100.0));
        check(compiled.value.has_value()
            && !compiled.value->matchesRevision(document.contentRevision()),
            "document mutation makes old snapshot stale");

        OperationStatus backgroundCaptureStatus = OperationStatus::Success;
        std::thread invalidCaptureThread([&document, &builder, &backgroundCaptureStatus]
        {
            backgroundCaptureStatus = builder.capture
                (document, context(QStringLiteral("invalid-background-capture"))).status;
        });
        invalidCaptureThread.join();
        check(backgroundCaptureStatus == OperationStatus::Failed,
            "background thread cannot capture active CadDocument");
    }

    void testDetachedParallelCompilation()
    {
        GeometrySourceSnapshot detachedSource;
        {
            auto document = std::make_unique<CadDocument>();
            detachedSource = captureMixedDocument(*document);
        }
        const OperationResult<GeometrySnapshot> compiled = compileSnapshot
            (detachedSource, GeometryExecutionMode::Parallel);
        check(compiled.status == OperationStatus::Success
            && compiled.value.has_value() && compiled.value->entries.size() == 7U,
            "parallel compiler uses detached values after document destruction");
    }
}

int runGeometrySnapshotTests()
{
    failures = 0;
    testCaptureCompileAndParity();
    testPartialFailureAndCancellation();
    testRevisionAndThreadBoundaries();
    testDetachedParallelCompilation();
    return failures;
}
