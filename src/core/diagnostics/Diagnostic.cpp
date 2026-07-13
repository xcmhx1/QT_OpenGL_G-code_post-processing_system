#include "pch.h"

#include "core/diagnostics/Diagnostic.h"

bool isErrorSeverity(DiagnosticSeverity severity)
{
    return severity == DiagnosticSeverity::Error
        || severity == DiagnosticSeverity::Critical;
}

QString diagnosticCodeName(DiagnosticCode code)
{
    switch (code)
    {
    case DiagnosticCode::None: return QStringLiteral("None");
    case DiagnosticCode::InvalidArgument: return QStringLiteral("InvalidArgument");
    case DiagnosticCode::MissingDocument: return QStringLiteral("MissingDocument");
    case DiagnosticCode::MissingProfile: return QStringLiteral("MissingProfile");
    case DiagnosticCode::InvalidGeometry: return QStringLiteral("InvalidGeometry");
    case DiagnosticCode::EmptyPath: return QStringLiteral("EmptyPath");
    case DiagnosticCode::MissingProcessOrder: return QStringLiteral("MissingProcessOrder");
    case DiagnosticCode::InvalidContinuousGroup: return QStringLiteral("InvalidContinuousGroup");
    case DiagnosticCode::KinematicsFailure: return QStringLiteral("KinematicsFailure");
    case DiagnosticCode::OvercutFailure: return QStringLiteral("OvercutFailure");
    case DiagnosticCode::FileOpenFailure: return QStringLiteral("FileOpenFailure");
    case DiagnosticCode::FileWriteFailure: return QStringLiteral("FileWriteFailure");
    case DiagnosticCode::OutputVerificationFailure: return QStringLiteral("OutputVerificationFailure");
    case DiagnosticCode::UnsupportedGeometry: return QStringLiteral("UnsupportedGeometry");
    case DiagnosticCode::InvalidSamplingPolicy: return QStringLiteral("InvalidSamplingPolicy");
    case DiagnosticCode::DegenerateGeometry: return QStringLiteral("DegenerateGeometry");
    case DiagnosticCode::GeometryAdapterFailure: return QStringLiteral("GeometryAdapterFailure");
    case DiagnosticCode::GeometryCompilationFailure: return QStringLiteral("GeometryCompilationFailure");
    case DiagnosticCode::PathInvariantViolation: return QStringLiteral("PathInvariantViolation");
    case DiagnosticCode::InternalInvariantViolation: return QStringLiteral("InternalInvariantViolation");
    }

    return QStringLiteral("Unknown");
}

QString diagnosticSeverityName(DiagnosticSeverity severity)
{
    switch (severity)
    {
    case DiagnosticSeverity::Trace: return QStringLiteral("Trace");
    case DiagnosticSeverity::Debug: return QStringLiteral("Debug");
    case DiagnosticSeverity::Info: return QStringLiteral("Info");
    case DiagnosticSeverity::Notice: return QStringLiteral("Notice");
    case DiagnosticSeverity::Warning: return QStringLiteral("Warning");
    case DiagnosticSeverity::Error: return QStringLiteral("Error");
    case DiagnosticSeverity::Critical: return QStringLiteral("Critical");
    }

    return QStringLiteral("Unknown");
}
