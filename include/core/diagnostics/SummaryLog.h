#pragma once

#include <QDebug>
#include <QString>

namespace cadcam::core
{
    // 结构化汇总日志的统一出口：
    // component 与 event 决定输出前缀，message 为渲染后的单行内容。
    // event 为空时输出单括号前缀 [component]，保持既有单事件日志格式。
    // 全部阶段汇总日志必须经过此出口，便于后续接入结构化日志接收器。
    inline void emitSummaryLog
    (
        const QString& component,
        const QString& event,
        const QString& message
    )
    {
        const QString prefix = event.isEmpty()
            ? QStringLiteral("[%1] ").arg(component)
            : QStringLiteral("[%1][%2] ").arg(component, event);
        qInfo().noquote() << prefix + message;
    }
}
