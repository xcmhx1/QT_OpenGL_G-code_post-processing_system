#pragma once

#include <QString>
#include <QVariantMap>

#include <optional>

enum class OperationStatus
{
    Success,
    PartialSuccess,
    Cancelled,
    InvalidInput,
    NotSupported,
    Conflict,
    Failed,
    InternalError
};

enum class DiagnosticSeverity
{
    Trace,
    Debug,
    Info,
    Notice,
    Warning,
    Error,
    Critical
};

enum class DiagnosticCode
{
    None,
    InvalidArgument,
    MissingDocument,
    MissingProfile,
    InvalidGeometry,
    EmptyPath,
    MissingProcessOrder,
    InvalidContinuousGroup,
    KinematicsFailure,
    OvercutFailure,
    FileOpenFailure,
    FileWriteFailure,
    OutputVerificationFailure,
    InternalInvariantViolation
};

struct Diagnostic
{
    DiagnosticCode code = DiagnosticCode::None;
    DiagnosticSeverity severity = DiagnosticSeverity::Info;
    QString component;
    QString operation;
    QString stage;
    QString userMessage;
    QString technicalDetail;
    QString correlationId;
    std::optional<quint64> entityId;
    std::optional<int> groupId;
    QVariantMap context;
};

bool isErrorSeverity(DiagnosticSeverity severity);
QString diagnosticCodeName(DiagnosticCode code);
QString diagnosticSeverityName(DiagnosticSeverity severity);

