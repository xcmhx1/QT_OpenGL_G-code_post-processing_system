#pragma once

#include "core/diagnostics/OperationContext.h"

#include <atomic>
#include <cstddef>
#include <functional>
#include <memory>

class CancellationToken
{
public:
    CancellationToken() = default;

    bool isCancellationRequested() const
    {
        return m_cancelled != nullptr && m_cancelled->load(std::memory_order_acquire);
    }

private:
    explicit CancellationToken(std::shared_ptr<std::atomic_bool> cancelled)
        : m_cancelled(std::move(cancelled))
    {
    }

    std::shared_ptr<std::atomic_bool> m_cancelled;
    friend class CancellationSource;
};

class CancellationSource
{
public:
    CancellationSource()
        : m_cancelled(std::make_shared<std::atomic_bool>(false))
    {
    }

    CancellationToken token() const
    {
        return CancellationToken(m_cancelled);
    }

    void cancel() const
    {
        m_cancelled->store(true, std::memory_order_release);
    }

private:
    std::shared_ptr<std::atomic_bool> m_cancelled;
};

struct GeometryBuildProgress
{
    std::size_t completedCount = 0;
    std::size_t totalCount = 0;
};

struct TaskContext
{
    OperationContext operationContext;
    CancellationToken cancellationToken;
    std::function<void(const GeometryBuildProgress&)> progressCallback;
    int maximumWorkerCount = 0;

    void reportProgress(std::size_t completedCount, std::size_t totalCount) const
    {
        if (progressCallback)
        {
            progressCallback({ completedCount, totalCount });
        }
    }
};
