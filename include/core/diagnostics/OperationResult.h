#pragma once

#include "core/diagnostics/Diagnostic.h"

#include <QVector>

#include <optional>
#include <variant>

template<typename T>
struct OperationResult
{
    OperationStatus status = OperationStatus::InternalError;
    std::optional<T> value;
    QVector<Diagnostic> diagnostics;

    bool succeeded() const
    {
        return status == OperationStatus::Success
            || status == OperationStatus::PartialSuccess;
    }

    bool hasErrors() const
    {
        for (const Diagnostic& diagnostic : diagnostics)
        {
            if (isErrorSeverity(diagnostic.severity))
            {
                return true;
            }
        }

        return false;
    }

    void addDiagnostic(const Diagnostic& diagnostic)
    {
        diagnostics.push_back(diagnostic);
    }

    void mergeDiagnostics(const QVector<Diagnostic>& otherDiagnostics)
    {
        diagnostics += otherDiagnostics;
    }

    template<typename U>
    void mergeDiagnostics(const OperationResult<U>& other)
    {
        mergeDiagnostics(other.diagnostics);
    }
};

using OperationReport = OperationResult<std::monostate>;

