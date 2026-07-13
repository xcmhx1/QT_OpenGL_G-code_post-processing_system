#pragma once

#include "application/messaging/MessageCenter.h"

#include <functional>

class UserInterfaceMessageSink : public IMessageSink
{
public:
    using CommandCallback = std::function<void(const QString&)>;
    using StatusCallback = std::function<void(const QString&, int)>;
    using DialogCallback = std::function<void(const QString&, const QString&)>;

    UserInterfaceMessageSink
    (
        CommandCallback commandCallback,
        StatusCallback statusCallback,
        DialogCallback dialogCallback,
        bool showBlockingErrors
    );

    void publish(const Diagnostic& diagnostic) override;

private:
    CommandCallback m_commandCallback;
    StatusCallback m_statusCallback;
    DialogCallback m_dialogCallback;
    bool m_showBlockingErrors = false;
};

