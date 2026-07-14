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
    case DiagnosticCode::InvalidPolyline: return QStringLiteral("InvalidPolyline");
    case DiagnosticCode::InvalidPolylineVertex: return QStringLiteral("InvalidPolylineVertex");
    case DiagnosticCode::InvalidBulge: return QStringLiteral("InvalidBulge");
    case DiagnosticCode::PolylinePlaneFailure: return QStringLiteral("PolylinePlaneFailure");
    case DiagnosticCode::InvalidSpline: return QStringLiteral("InvalidSpline");
    case DiagnosticCode::InvalidSplineDegree: return QStringLiteral("InvalidSplineDegree");
    case DiagnosticCode::InvalidSplineControlPoints: return QStringLiteral("InvalidSplineControlPoints");
    case DiagnosticCode::InvalidSplineKnots: return QStringLiteral("InvalidSplineKnots");
    case DiagnosticCode::InvalidSplineWeights: return QStringLiteral("InvalidSplineWeights");
    case DiagnosticCode::InvalidSplineParameterDomain: return QStringLiteral("InvalidSplineParameterDomain");
    case DiagnosticCode::SplineEvaluationFailure: return QStringLiteral("SplineEvaluationFailure");
    case DiagnosticCode::SplineSubdivisionLimit: return QStringLiteral("SplineSubdivisionLimit");
    case DiagnosticCode::SplinePointLimitExceeded: return QStringLiteral("SplinePointLimitExceeded");
    case DiagnosticCode::SplineFitPointFallbackUsed: return QStringLiteral("SplineFitPointFallbackUsed");
    case DiagnosticCode::SplineProductionCompileFailure: return QStringLiteral("SplineProductionCompileFailure");
    case DiagnosticCode::SplineLegacyFallbackUsed: return QStringLiteral("SplineLegacyFallbackUsed");
    case DiagnosticCode::SplineLegacyParityMismatch: return QStringLiteral("SplineLegacyParityMismatch");
    case DiagnosticCode::SplineDisplayPathFailure: return QStringLiteral("SplineDisplayPathFailure");
    case DiagnosticCode::SplineControlPointFailure: return QStringLiteral("SplineControlPointFailure");
    case DiagnosticCode::TopologyInputInvalid: return QStringLiteral("TopologyInputInvalid");
    case DiagnosticCode::TopologyPathUnavailable: return QStringLiteral("TopologyPathUnavailable");
    case DiagnosticCode::TopologyBuildFailure: return QStringLiteral("TopologyBuildFailure");
    case DiagnosticCode::TopologySeedNotFound: return QStringLiteral("TopologySeedNotFound");
    case DiagnosticCode::TopologyLoopNotFound: return QStringLiteral("TopologyLoopNotFound");
    case DiagnosticCode::TopologyBranchIgnored: return QStringLiteral("TopologyBranchIgnored");
    case DiagnosticCode::TopologyLoopDiscontinuous: return QStringLiteral("TopologyLoopDiscontinuous");
    case DiagnosticCode::TopologyParityMismatch: return QStringLiteral("TopologyParityMismatch");
    case DiagnosticCode::LegacyTopologyAdapterFailure: return QStringLiteral("LegacyTopologyAdapterFailure");
    case DiagnosticCode::DuplicateTopologyEntityId: return QStringLiteral("DuplicateTopologyEntityId");
    case DiagnosticCode::TopologyResultMappingFailure: return QStringLiteral("TopologyResultMappingFailure");
    case DiagnosticCode::CutBoundaryTopologyInvalid: return QStringLiteral("CutBoundaryTopologyInvalid");
    case DiagnosticCode::CutBoundarySectionUnavailable: return QStringLiteral("CutBoundarySectionUnavailable");
    case DiagnosticCode::CutBoundarySurfaceMappingFailed: return QStringLiteral("CutBoundarySurfaceMappingFailed");
    case DiagnosticCode::CutBoundaryProjectionMismatch: return QStringLiteral("CutBoundaryProjectionMismatch");
    case DiagnosticCode::CutBoundaryCoverageGap: return QStringLiteral("CutBoundaryCoverageGap");
    case DiagnosticCode::CutBoundaryCenterMismatch: return QStringLiteral("CutBoundaryCenterMismatch");
    case DiagnosticCode::CutBoundaryDimensionMismatch: return QStringLiteral("CutBoundaryDimensionMismatch");
    case DiagnosticCode::CutBoundarySeamDegenerate: return QStringLiteral("CutBoundarySeamDegenerate");
    case DiagnosticCode::CutBoundarySeamDisagreement: return QStringLiteral("CutBoundarySeamDisagreement");
    case DiagnosticCode::CutBoundaryWindingMismatch: return QStringLiteral("CutBoundaryWindingMismatch");
    case DiagnosticCode::CutBoundaryMultipleWinding: return QStringLiteral("CutBoundaryMultipleWinding");
    case DiagnosticCode::CutBoundaryKeepsTubeConnected: return QStringLiteral("CutBoundaryKeepsTubeConnected");
    case DiagnosticCode::TubeSectionInputInvalid: return QStringLiteral("TubeSectionInputInvalid");
    case DiagnosticCode::TubeSectionLoopUnavailable: return QStringLiteral("TubeSectionLoopUnavailable");
    case DiagnosticCode::TubeSectionNotPerpendicular: return QStringLiteral("TubeSectionNotPerpendicular");
    case DiagnosticCode::TubeSectionBoundaryInvalid: return QStringLiteral("TubeSectionBoundaryInvalid");
    case DiagnosticCode::TubeSectionSelfIntersection: return QStringLiteral("TubeSectionSelfIntersection");
    case DiagnosticCode::TubeSectionAreaInvalid: return QStringLiteral("TubeSectionAreaInvalid");
    case DiagnosticCode::TubeSectionMultipleOuterLoops: return QStringLiteral("TubeSectionMultipleOuterLoops");
    case DiagnosticCode::TubeSectionPreparationFailed: return QStringLiteral("TubeSectionPreparationFailed");
    case DiagnosticCode::TubeSectionInteriorClassificationFailed: return QStringLiteral("TubeSectionInteriorClassificationFailed");
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
