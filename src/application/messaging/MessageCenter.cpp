#include "pch.h"

#include "application/messaging/MessageCenter.h"

void MessageCenter::addSink(IMessageSink* sink)
{
    if (sink != nullptr && !m_sinks.contains(sink))
    {
        m_sinks.push_back(sink);
    }
}

void MessageCenter::removeSink(IMessageSink* sink)
{
    m_sinks.removeAll(sink);
}

void MessageCenter::publish(const Diagnostic& diagnostic) const
{
    for (IMessageSink* sink : m_sinks)
    {
        if (sink != nullptr)
        {
            sink->publish(diagnostic);
        }
    }
}

void MessageCenter::publish(const OperationReport& report) const
{
    for (const Diagnostic& diagnostic : report.diagnostics)
    {
        publish(diagnostic);
    }
}

