#pragma once

#include "core/diagnostics/OperationResult.h"

#include <QVector>

class IMessageSink
{
public:
    virtual ~IMessageSink() = default;
    virtual void publish(const Diagnostic& diagnostic) = 0;
};

class MessageCenter
{
public:
    void addSink(IMessageSink* sink);
    void removeSink(IMessageSink* sink);

    void publish(const Diagnostic& diagnostic) const;
    void publish(const OperationReport& report) const;

    template<typename T>
    void publish(const OperationResult<T>& result) const
    {
        for (const Diagnostic& diagnostic : result.diagnostics)
        {
            publish(diagnostic);
        }
    }

private:
    QVector<IMessageSink*> m_sinks;
};
