#include "platform/pch.h"

#include "application/messaging/DebugMessageSink.h"

#include <QDebug>

void DebugMessageSink::publish(const Diagnostic& diagnostic)
{
    const QString message = QStringLiteral("[%1] [%2] [%3] %4 | %5")
        .arg(diagnostic.correlationId)
        .arg(diagnostic.component)
        .arg(diagnosticCodeName(diagnostic.code))
        .arg(diagnostic.userMessage)
        .arg(diagnostic.technicalDetail);

    switch (diagnostic.severity)
    {
    case DiagnosticSeverity::Trace:
    case DiagnosticSeverity::Debug:
        qDebug().noquote() << message << diagnostic.context;
        break;
    case DiagnosticSeverity::Info:
    case DiagnosticSeverity::Notice:
        qInfo().noquote() << message << diagnostic.context;
        break;
    case DiagnosticSeverity::Warning:
        qWarning().noquote() << message << diagnostic.context;
        break;
    case DiagnosticSeverity::Error:
    case DiagnosticSeverity::Critical:
        qCritical().noquote() << message << diagnostic.context;
        break;
    }
}

