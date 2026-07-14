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
    case DiagnosticCode::ProcessPlanningInputInvalid: return QStringLiteral("ProcessPlanningInputInvalid");
    case DiagnosticCode::ProcessPlanningRevisionMismatch: return QStringLiteral("ProcessPlanningRevisionMismatch");
    case DiagnosticCode::ProcessPlanningNoProcessableEntities: return QStringLiteral("ProcessPlanningNoProcessableEntities");
    case DiagnosticCode::ProcessPlanningBoundaryInvalid: return QStringLiteral("ProcessPlanningBoundaryInvalid");
    case DiagnosticCode::ProcessPlanningBoundaryKeepsConnected: return QStringLiteral("ProcessPlanningBoundaryKeepsConnected");
    case DiagnosticCode::ProcessPlanningBoundaryClassificationFailed: return QStringLiteral("ProcessPlanningBoundaryClassificationFailed");
    case DiagnosticCode::ProcessPlanningPrecedenceCycle: return QStringLiteral("ProcessPlanningPrecedenceCycle");
    case DiagnosticCode::ProcessPlanningGroupBuildFailed: return QStringLiteral("ProcessPlanningGroupBuildFailed");
    case DiagnosticCode::ProcessPlanningDirectionFailed: return QStringLiteral("ProcessPlanningDirectionFailed");
    case DiagnosticCode::ProcessPlanningOrderingFailed: return QStringLiteral("ProcessPlanningOrderingFailed");
    case DiagnosticCode::ProcessPlanningInvariantViolation: return QStringLiteral("ProcessPlanningInvariantViolation");
    case DiagnosticCode::ProcessPlanApplyConflict: return QStringLiteral("ProcessPlanApplyConflict");
    case DiagnosticCode::ProcessPlanEntityMissing: return QStringLiteral("ProcessPlanEntityMissing");
    case DiagnosticCode::ProcessPlanModeMismatch: return QStringLiteral("ProcessPlanModeMismatch");
    case DiagnosticCode::ProcessStateEntityMissing: return QStringLiteral("ProcessStateEntityMissing");
    case DiagnosticCode::ProcessStateRevisionMismatch: return QStringLiteral("ProcessStateRevisionMismatch");
    case DiagnosticCode::ProcessStateInvalidOverride: return QStringLiteral("ProcessStateInvalidOverride");
    case DiagnosticCode::ProcessPresentationInvalid: return QStringLiteral("ProcessPresentationInvalid");
    case DiagnosticCode::ProcessPresentationRevisionMismatch: return QStringLiteral("ProcessPresentationRevisionMismatch");
    case DiagnosticCode::PlanarPlanningInputInvalid: return QStringLiteral("PlanarPlanningInputInvalid");
    case DiagnosticCode::PlanarPlanningNoProcessableEntities: return QStringLiteral("PlanarPlanningNoProcessableEntities");
    case DiagnosticCode::PlanarPlanningOrderingFailed: return QStringLiteral("PlanarPlanningOrderingFailed");
    case DiagnosticCode::MachineTrajectoryInputInvalid: return QStringLiteral("MachineTrajectoryInputInvalid");
    case DiagnosticCode::MachineTrajectoryRevisionMismatch: return QStringLiteral("MachineTrajectoryRevisionMismatch");
    case DiagnosticCode::MachineTrajectoryEntityMissing: return QStringLiteral("MachineTrajectoryEntityMissing");
    case DiagnosticCode::MachineTrajectoryGeometryCompileFailed: return QStringLiteral("MachineTrajectoryGeometryCompileFailed");
    case DiagnosticCode::MachineTrajectoryInvalidPath: return QStringLiteral("MachineTrajectoryInvalidPath");
    case DiagnosticCode::RotaryKinematicsFailed: return QStringLiteral("RotaryKinematicsFailed");
    case DiagnosticCode::RotaryCenterInvalid: return QStringLiteral("RotaryCenterInvalid");
    case DiagnosticCode::RotaryCornerGeometryInvalid: return QStringLiteral("RotaryCornerGeometryInvalid");
    case DiagnosticCode::MachineTrajectoryContinuityFailure: return QStringLiteral("MachineTrajectoryContinuityFailure");
    case DiagnosticCode::MachineTrajectorySafeMoveFailed: return QStringLiteral("MachineTrajectorySafeMoveFailed");
    case DiagnosticCode::MachineTrajectoryOvercutFailed: return QStringLiteral("MachineTrajectoryOvercutFailed");
    case DiagnosticCode::MachineTrajectoryInvariantViolation: return QStringLiteral("MachineTrajectoryInvariantViolation");
    case DiagnosticCode::NcProgramInputInvalid: return QStringLiteral("NcProgramInputInvalid");
    case DiagnosticCode::NcProgramRevisionMismatch: return QStringLiteral("NcProgramRevisionMismatch");
    case DiagnosticCode::NcProgramEntityMissing: return QStringLiteral("NcProgramEntityMissing");
    case DiagnosticCode::NcProgramMetadataMissing: return QStringLiteral("NcProgramMetadataMissing");
    case DiagnosticCode::NcProgramDuplicateEntity: return QStringLiteral("NcProgramDuplicateEntity");
    case DiagnosticCode::NcProgramUnsupportedMotion: return QStringLiteral("NcProgramUnsupportedMotion");
    case DiagnosticCode::NcProgramInvariantViolation: return QStringLiteral("NcProgramInvariantViolation");
    case DiagnosticCode::GCodeProfileInvalid: return QStringLiteral("GCodeProfileInvalid");
    case DiagnosticCode::GCodeEntityBlockMissing: return QStringLiteral("GCodeEntityBlockMissing");
    case DiagnosticCode::GCodeRenderingFailed: return QStringLiteral("GCodeRenderingFailed");
    case DiagnosticCode::GCodeTextOptimizationFailed: return QStringLiteral("GCodeTextOptimizationFailed");
    case DiagnosticCode::PlanarNcInputInvalid: return QStringLiteral("PlanarNcInputInvalid");
    case DiagnosticCode::PlanarNcRevisionMismatch: return QStringLiteral("PlanarNcRevisionMismatch");
    case DiagnosticCode::PlanarNcEntityMissing: return QStringLiteral("PlanarNcEntityMissing");
    case DiagnosticCode::PlanarNcUnsupportedGeometry: return QStringLiteral("PlanarNcUnsupportedGeometry");
    case DiagnosticCode::PlanarNcGeometryCompileFailed: return QStringLiteral("PlanarNcGeometryCompileFailed");
    case DiagnosticCode::PlanarNcInvalidArcPlane: return QStringLiteral("PlanarNcInvalidArcPlane");
    case DiagnosticCode::PlanarNcInvalidMotion: return QStringLiteral("PlanarNcInvalidMotion");
    case DiagnosticCode::PlanarNcInvariantViolation: return QStringLiteral("PlanarNcInvariantViolation");
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
