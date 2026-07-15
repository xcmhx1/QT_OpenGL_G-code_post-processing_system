#include "platform/pch.h"

#include "core/diagnostics/OperationContext.h"

#include <QDate>

#include <atomic>

OperationContext createOperationContext(const QString& operationName)
{
    static std::atomic<quint64> sequence{ 0 };

    QString normalizedName = operationName.trimmed().toLower();
    normalizedName.replace(QLatin1Char(' '), QLatin1Char('-'));

    if (normalizedName.isEmpty())
    {
        normalizedName = QStringLiteral("operation");
    }

    const quint64 nextSequence = ++sequence;
    return
    {
        QStringLiteral("%1-%2-%3")
            .arg(normalizedName)
            .arg(QDate::currentDate().toString(QStringLiteral("yyyyMMdd")))
            .arg(nextSequence, 4, 10, QLatin1Char('0')),
        normalizedName
    };
}

