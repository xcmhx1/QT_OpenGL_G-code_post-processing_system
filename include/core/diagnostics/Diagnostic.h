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
    UnsupportedGeometry,
    InvalidSamplingPolicy,
    DegenerateGeometry,
    GeometryAdapterFailure,
    GeometryCompilationFailure,
    PathInvariantViolation,
    InvalidPolyline,
    InvalidPolylineVertex,
    InvalidBulge,
    PolylinePlaneFailure,
    InvalidSpline,
    InvalidSplineDegree,
    InvalidSplineControlPoints,
    InvalidSplineKnots,
    InvalidSplineWeights,
    InvalidSplineParameterDomain,
    SplineEvaluationFailure,
    SplineSubdivisionLimit,
    SplinePointLimitExceeded,
    SplineFitPointFallbackUsed,
    SplineProductionCompileFailure,
    SplineLegacyFallbackUsed,
    SplineLegacyParityMismatch,
    SplineDisplayPathFailure,
    SplineControlPointFailure,
    TopologyInputInvalid,
    TopologyPathUnavailable,
    TopologyBuildFailure,
    TopologySeedNotFound,
    TopologyLoopNotFound,
    TopologyBranchIgnored,
    TopologyLoopDiscontinuous,
    TopologyParityMismatch,
    LegacyTopologyAdapterFailure,
    DuplicateTopologyEntityId,
    TopologyResultMappingFailure,
    CutBoundaryTopologyInvalid,
    CutBoundarySectionUnavailable,
    CutBoundarySurfaceMappingFailed,
    CutBoundaryProjectionMismatch,
    CutBoundaryCoverageGap,
    CutBoundaryCenterMismatch,
    CutBoundaryDimensionMismatch,
    CutBoundarySeamDegenerate,
    CutBoundarySeamDisagreement,
    CutBoundaryWindingMismatch,
    CutBoundaryMultipleWinding,
    CutBoundaryKeepsTubeConnected,
    TubeSectionInputInvalid,
    TubeSectionLoopUnavailable,
    TubeSectionNotPerpendicular,
    TubeSectionBoundaryInvalid,
    TubeSectionSelfIntersection,
    TubeSectionAreaInvalid,
    TubeSectionMultipleOuterLoops,
    TubeSectionPreparationFailed,
    TubeSectionInteriorClassificationFailed,
    ProcessPlanningInputInvalid,
    ProcessPlanningRevisionMismatch,
    ProcessPlanningNoProcessableEntities,
    ProcessPlanningBoundaryInvalid,
    ProcessPlanningBoundaryKeepsConnected,
    ProcessPlanningBoundaryClassificationFailed,
    ProcessPlanningPrecedenceCycle,
    ProcessPlanningGroupBuildFailed,
    ProcessPlanningClosedLoopSummary,
    ProcessPlanningSurfaceSweepSummary,
    ProcessPlanningZone16Profile,
    ProcessPlanningZone16Summary,
    ProcessPlanningDirectionFailed,
    ProcessPlanningOrderingFailed,
    ProcessPlanningInvariantViolation,
    ProcessPlanApplyConflict,
    ProcessPlanEntityMissing,
    ProcessPlanModeMismatch,
    ProcessStateEntityMissing,
    ProcessStateRevisionMismatch,
    ProcessStateInvalidOverride,
    ProcessPresentationInvalid,
    ProcessPresentationRevisionMismatch,
    PlanarPlanningInputInvalid,
    PlanarPlanningNoProcessableEntities,
    PlanarPlanningOrderingFailed,
    MachineTrajectoryInputInvalid,
    MachineTrajectoryRevisionMismatch,
    MachineTrajectoryEntityMissing,
    MachineTrajectoryGeometryCompileFailed,
    MachineTrajectoryInvalidPath,
    RotaryKinematicsFailed,
    RotarySurfaceClassificationFailed,
    RotaryCenterInvalid,
    RotaryCornerGeometryInvalid,
    TubeSectionProjectionFailed,
    MachineTrajectoryContinuityFailure,
    MachineTrajectorySafeMoveFailed,
    MachineTrajectoryOvercutFailed,
    MachineTrajectoryInvariantViolation,
    NcProgramInputInvalid,
    NcProgramRevisionMismatch,
    NcProgramEntityMissing,
    NcProgramMetadataMissing,
    NcProgramDuplicateEntity,
    NcProgramUnsupportedMotion,
    NcProgramInvariantViolation,
    GCodeProfileInvalid,
    GCodeEntityBlockMissing,
    GCodeRenderingFailed,
    GCodeTextOptimizationFailed,
    PlanarNcInputInvalid,
    PlanarNcRevisionMismatch,
    PlanarNcEntityMissing,
    PlanarNcUnsupportedGeometry,
    PlanarNcGeometryCompileFailed,
    PlanarNcInvalidArcPlane,
    PlanarNcInvalidMotion,
    PlanarNcInvariantViolation,
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
