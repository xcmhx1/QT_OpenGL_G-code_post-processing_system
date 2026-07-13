#include "pch.h"

#include "application/messaging/UserInterfaceMessageSink.h"

UserInterfaceMessageSink::UserInterfaceMessageSink
(
    CommandCallback commandCallback,
    StatusCallback statusCallback,
    DialogCallback dialogCallback,
    bool showBlockingErrors
)
    : m_commandCallback(std::move(commandCallback))
    , m_statusCallback(std::move(statusCallback))
    , m_dialogCallback(std::move(dialogCallback))
    , m_showBlockingErrors(showBlockingErrors)
{
}

void UserInterfaceMessageSink::publish(const Diagnostic& diagnostic)
{
    const QString message = diagnostic.userMessage.trimmed().isEmpty()
        ? diagnostic.technicalDetail
        : diagnostic.userMessage;

    switch (diagnostic.severity)
    {
    case DiagnosticSeverity::Trace:
    case DiagnosticSeverity::Debug:
        return;

    case DiagnosticSeverity::Info:
    case DiagnosticSeverity::Notice:
        if (m_commandCallback)
        {
            m_commandCallback(message);
        }
        if (m_statusCallback && diagnostic.context.contains(QStringLiteral("statusDurationMs")))
        {
            m_statusCallback
            (
                diagnostic.context.value(QStringLiteral("statusMessage"), message).toString(),
                diagnostic.context.value(QStringLiteral("statusDurationMs")).toInt()
            );
        }
        return;

    case DiagnosticSeverity::Warning:
        if (m_commandCallback)
        {
            m_commandCallback(message);
        }
        if (m_statusCallback)
        {
            m_statusCallback(message, 5000);
        }
        return;

    case DiagnosticSeverity::Error:
        if (m_commandCallback)
        {
            m_commandCallback(message);
        }
        if (m_statusCallback)
        {
            m_statusCallback(message, 8000);
        }
        if (m_showBlockingErrors && m_dialogCallback)
        {
            m_dialogCallback(QStringLiteral("操作失败"), message);
        }
        return;

    case DiagnosticSeverity::Critical:
        if (m_commandCallback)
        {
            m_commandCallback(message);
        }
        if (m_statusCallback)
        {
            m_statusCallback(message, 10000);
        }
        if (m_dialogCallback)
        {
            m_dialogCallback(QStringLiteral("严重错误"), message);
        }
        return;
    }
}
