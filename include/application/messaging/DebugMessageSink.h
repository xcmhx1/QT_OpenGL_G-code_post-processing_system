#pragma once

#include "application/messaging/MessageCenter.h"

class DebugMessageSink : public IMessageSink
{
public:
    void publish(const Diagnostic& diagnostic) override;
};

