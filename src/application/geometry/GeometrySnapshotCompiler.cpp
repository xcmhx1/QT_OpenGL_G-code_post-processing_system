#include "application/geometry/GeometrySnapshotCompiler.h"

#include <QThread>
#include <QThreadPool>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <mutex>

namespace
{
    Diagnostic snapshotDiagnostic
    (
        const OperationContext& context,
        DiagnosticCode code,
        DiagnosticSeverity severity,
        const QString& detail
    )
    {
        Diagnostic diagnostic;
        diagnostic.code = code;
        diagnostic.severity = severity;
        diagnostic.component = QStringLiteral("GeometrySnapshotCompiler");
        diagnostic.operation = QStringLiteral("CompileGeometrySnapshot");
        diagnostic.stage = QStringLiteral("CompileSourceEntry");
        diagnostic.userMessage = code == DiagnosticCode::InternalInvariantViolation
            ? QStringLiteral("几何快照编译器内部状态无效。")
            : QStringLiteral("部分几何图元无法编译为路径快照。");
        diagnostic.technicalDetail = detail;
        diagnostic.correlationId = context.correlationId;
        return diagnostic;
    }

    void enrichDiagnostics
    (
        QVector<Diagnostic>& diagnostics,
        const GeometrySnapshotEntry& entry
    )
    {
        const QString kindName = QString::fromLatin1
            (cadcam::geometry::sourceGeometryKindName(entry.sourceKind));
        for (Diagnostic& diagnostic : diagnostics)
        {
            diagnostic.entityId = entry.attributes.entityId;
            diagnostic.context.insert
                (QStringLiteral("sourceIndex"), static_cast<qulonglong>(entry.sourceIndex));
            diagnostic.context.insert(QStringLiteral("sourceKind"), kindName);
        }
    }

    GeometrySnapshotEntry compileEntry
    (
        const GeometrySourceEntry& sourceEntry,
        const cadcam::geometry::SamplingPolicy& policy,
        const OperationContext& context
    )
    {
        GeometrySnapshotEntry entry;
        entry.sourceIndex = sourceEntry.sourceIndex;
        entry.attributes = sourceEntry.attributes;
        entry.sourceKind = sourceEntry.sourceKind;
        entry.status = sourceEntry.status;
        entry.diagnostics = sourceEntry.diagnostics;

        if (!sourceEntry.sourceEntity.has_value())
        {
            if (entry.diagnostics.isEmpty())
            {
                entry.diagnostics.push_back(snapshotDiagnostic
                (
                    context,
                    DiagnosticCode::GeometryCompilationFailure,
                    DiagnosticSeverity::Error,
                    QStringLiteral("source entry does not contain SourceEntity")
                ));
            }
            if (entry.status == OperationStatus::Success)
            {
                entry.status = OperationStatus::Failed;
            }
            enrichDiagnostics(entry.diagnostics, entry);
            return entry;
        }

        cadcam::geometry::PathCompileOptions options;
        options.reverse = false;
        options.startParameter = std::nullopt;
        cadcam::geometry::GeometryCompiler compiler;
        OperationResult<cadcam::geometry::Path3D> compiled = compiler.compile
        (
            *sourceEntry.sourceEntity,
            policy,
            options,
            context
        );
        entry.diagnostics += compiled.diagnostics;
        entry.status = sourceEntry.status == OperationStatus::PartialSuccess
            || compiled.status == OperationStatus::PartialSuccess
            ? OperationStatus::PartialSuccess
            : compiled.status;
        if (compiled.value.has_value())
        {
            entry.path = std::move(*compiled.value);
        }
        if (!compiled.succeeded() && entry.diagnostics.isEmpty())
        {
            entry.diagnostics.push_back(snapshotDiagnostic
            (
                context,
                DiagnosticCode::GeometryCompilationFailure,
                DiagnosticSeverity::Error,
                QStringLiteral("GeometryCompiler returned no path")
            ));
        }
        enrichDiagnostics(entry.diagnostics, entry);
        return entry;
    }

    Diagnostic cancellationDiagnostic
    (
        const OperationContext& context,
        std::size_t completedCount,
        std::size_t totalCount
    )
    {
        Diagnostic diagnostic = snapshotDiagnostic
        (
            context,
            DiagnosticCode::None,
            DiagnosticSeverity::Notice,
            QStringLiteral("geometry snapshot compilation was cancelled")
        );
        diagnostic.userMessage = QStringLiteral("几何快照编译已取消。");
        diagnostic.context.insert
            (QStringLiteral("completedCount"), static_cast<qulonglong>(completedCount));
        diagnostic.context.insert
            (QStringLiteral("totalCount"), static_cast<qulonglong>(totalCount));
        return diagnostic;
    }

    OperationStatus completedStatus
    (
        const std::vector<GeometrySnapshotEntry>& entries,
        bool cancelled
    )
    {
        if (cancelled)
        {
            return OperationStatus::Cancelled;
        }
        const bool hasNonSuccess = std::any_of
        (
            entries.cbegin(), entries.cend(),
            [](const GeometrySnapshotEntry& entry)
            {
                return entry.status != OperationStatus::Success || !entry.path.has_value();
            }
        );
        return hasNonSuccess ? OperationStatus::PartialSuccess : OperationStatus::Success;
    }
}

OperationResult<GeometrySnapshot> GeometrySnapshotCompiler::compile
(
    const GeometrySourceSnapshot& source,
    const cadcam::geometry::SamplingPolicy& policy,
    GeometryExecutionMode mode,
    const TaskContext& taskContext
) const
{
    OperationResult<GeometrySnapshot> result;
    if (source.contentRevision == 0U)
    {
        result.status = OperationStatus::Failed;
        result.addDiagnostic(snapshotDiagnostic
        (
            taskContext.operationContext,
            DiagnosticCode::InternalInvariantViolation,
            DiagnosticSeverity::Error,
            QStringLiteral("source snapshot revision is zero")
        ));
        return result;
    }

    GeometrySnapshot snapshot;
    snapshot.contentRevision = source.contentRevision;
    snapshot.samplingPolicy = policy;
    const std::size_t totalCount = source.entries.size();
    taskContext.reportProgress(0U, totalCount);

    if (mode == GeometryExecutionMode::Serial)
    {
        snapshot.entries.reserve(totalCount);
        for (const GeometrySourceEntry& sourceEntry : source.entries)
        {
            if (taskContext.cancellationToken.isCancellationRequested())
            {
                break;
            }
            snapshot.entries.push_back
                (compileEntry(sourceEntry, policy, taskContext.operationContext));
            taskContext.reportProgress(snapshot.entries.size(), totalCount);
            if (taskContext.cancellationToken.isCancellationRequested())
            {
                break;
            }
        }
    }
    else if (totalCount > 0U
        && !taskContext.cancellationToken.isCancellationRequested())
    {
        int workerCount = std::max(1, QThread::idealThreadCount() - 1);
        if (taskContext.maximumWorkerCount > 0)
        {
            workerCount = std::min(workerCount, taskContext.maximumWorkerCount);
        }
        workerCount = std::min(workerCount, static_cast<int>(totalCount));

        QThreadPool pool;
        pool.setMaxThreadCount(workerCount);
        pool.setExpiryTimeout(-1);
        std::vector<std::optional<GeometrySnapshotEntry>> resultEntries(totalCount);
        std::atomic_size_t nextIndex{ 0U };
        std::atomic_size_t completedCount{ 0U };
        std::atomic_int activeWorkers{ workerCount };
        std::mutex completionMutex;
        std::condition_variable completionChanged;

        for (int worker = 0; worker < workerCount; ++worker)
        {
            pool.start
            ([&source, &policy, &taskContext, &resultEntries, &nextIndex,
                &completedCount, &activeWorkers, &completionChanged]()
            {
                while (!taskContext.cancellationToken.isCancellationRequested())
                {
                    const std::size_t index = nextIndex.fetch_add(1U);
                    if (index >= source.entries.size()
                        || taskContext.cancellationToken.isCancellationRequested())
                    {
                        break;
                    }
                    resultEntries[index] = compileEntry
                        (source.entries[index], policy, taskContext.operationContext);
                    completedCount.fetch_add(1U);
                    completionChanged.notify_one();
                }
                activeWorkers.fetch_sub(1);
                completionChanged.notify_one();
            });
        }

        std::size_t reportedCount = 0U;
        std::unique_lock<std::mutex> lock(completionMutex);
        while (activeWorkers.load() > 0)
        {
            completionChanged.wait(lock, [&completedCount, &reportedCount, &activeWorkers]
            {
                return completedCount.load() != reportedCount || activeWorkers.load() == 0;
            });
            const std::size_t currentCount = completedCount.load();
            if (currentCount != reportedCount)
            {
                reportedCount = currentCount;
                taskContext.reportProgress(reportedCount, totalCount);
            }
        }
        lock.unlock();
        pool.waitForDone();

        snapshot.entries.reserve(completedCount.load());
        for (std::optional<GeometrySnapshotEntry>& entry : resultEntries)
        {
            if (entry.has_value())
            {
                snapshot.entries.push_back(std::move(*entry));
            }
        }
    }

    std::sort
    (
        snapshot.entries.begin(), snapshot.entries.end(),
        [](const GeometrySnapshotEntry& left, const GeometrySnapshotEntry& right)
        {
            return left.sourceIndex < right.sourceIndex;
        }
    );

    const bool cancelled = taskContext.cancellationToken.isCancellationRequested();
    result.status = completedStatus(snapshot.entries, cancelled);
    result.value = std::move(snapshot);
    for (const GeometrySnapshotEntry& entry : result.value->entries)
    {
        result.mergeDiagnostics(entry.diagnostics);
    }
    if (cancelled)
    {
        result.addDiagnostic(cancellationDiagnostic
        (
            taskContext.operationContext,
            result.value->entries.size(),
            totalCount
        ));
    }
    return result;
}
