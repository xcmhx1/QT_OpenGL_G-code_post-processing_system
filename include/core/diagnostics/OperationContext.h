#pragma once

#include <QString>

struct OperationContext
{
    QString correlationId;
    QString operationName;
};

OperationContext createOperationContext(const QString& operationName);

