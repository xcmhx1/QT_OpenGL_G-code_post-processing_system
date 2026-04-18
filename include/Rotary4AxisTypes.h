#pragma once

#include <QString>
#include <QVector>
#include <QVector2D>
#include <QVector3D>

enum class ProcessMode
{
    XYZ_3Axis,
    A_Indexed_XYZ,
    XYZA_Continuous
};

enum class SegmentMotionMode
{
    AFixed,
    AContinuous
};

enum class SectionRegionType
{
    Unknown,
    FlatSide,
    CornerTransition
};

struct RotaryConfig
{
    double yCenter = 0.0;
    double zCenter = 0.0;
    double aOffsetDeg = 0.0;
    bool aPositiveCCW = true;

    // A 角参考约定：
    // 1) 先把表面法向投影到 YZ 平面，得到 m=(ny,nz)。
    // 2) 再把 m 旋转到 targetNormalYZ 指定的“目标受光法向”方向。
    // 3) 该旋转量（含机床方向映射与零位偏移）即输出命令 A。
    // 默认 (0,1) 表示 A=0 时，法向目标朝 +Z。
    QVector2D targetNormalYZ = QVector2D(0.0f, 1.0f);
};

struct ProcessTolerance
{
    double nxThreshold = 0.25;
    double normalEps = 1.0e-6;
    double aConstTolDeg = 0.5;
    double aEnableThresholdDeg = 1.0;
    double aNeutralToleranceDeg = 0.6;
    double aRangeForContinuousDeg = 3.0;
    bool enableFlatSideQuadrantSnap = true;
    double flatSideSnapStepDeg = 90.0;
    double maxDeltaADegPerStep = 5.0;
    double maxLinearStep = 1.0;
    double fixedASwitchThresholdDeg = 0.5;
    double connectionTolerance = 0.05;
    double aJoinToleranceDeg = 0.5;
    double feedTolerance = 1.0;
    double powerTolerance = 1.0;
    bool allowReverseEntityForJoin = true;
    bool allowLaserOffNoLiftTransition = true;
};

struct SamplePoint
{
    QVector3D pos;
    QVector3D normal;
    bool hasNormal = false;
    bool hasRawA = false;
    double rawA = 0.0;
    bool hasSnappedA = false;
    double snappedA = 0.0;
    bool hasTargetA = false;
    double targetA = 0.0;
    SectionRegionType regionType = SectionRegionType::Unknown;
    int sideIndex = -1;
};

struct ReachabilityResult
{
    bool aOnlyReachable = true;
    double nxAbsMax = 0.0;
    bool hasDegenerateNormal = false;
};

struct ModeStatistics
{
    double aRangeDeg = 0.0;
    double maxDeltaADeg = 0.0;
    double maxDeltaARateDegPerUnit = 0.0;
};

struct ToolpathPoint4Axis
{
    QVector3D localPos;
    QVector3D machinePos;
    double aDeg = 0.0;
    double feed = 0.0;
    double power = 0.0;
    bool laserOn = false;
};

struct ToolpathSegment4Axis
{
    SegmentMotionMode mode = SegmentMotionMode::AContinuous;
    ProcessMode processMode = ProcessMode::XYZA_Continuous;
    QVector<ToolpathPoint4Axis> points;
    QString processKey;
    bool requiresA = true;
    bool hasFixedA = false;
    double fixedADeg = 0.0;
    double startADeg = 0.0;
    double endADeg = 0.0;
    double aRangeDeg = 0.0;
    bool emitAInEachLine = true;
    bool prePositionAOnly = false;
    QString regionSummary;
};

struct AttachedCurveSample
{
    QVector3D worldPos;
    QVector3D surfaceNormal;
    bool hasSurfaceNormal = false;
    double pathParam = 0.0;
};

struct MachineConfig4Axis
{
    QString rapidCode = QStringLiteral("G00");
    QString linearCode = QStringLiteral("G01");
    QString laserOnCode = QStringLiteral("M03");
    QString laserOffCode = QStringLiteral("M05");
    double feedDefault = 1200.0;
    double powerDefault = 100.0;
    int coordPrecision = 5;
    int anglePrecision = 5;
    int feedPrecision = 1;
    bool metricUnits = true;
    bool absoluteMode = true;
    bool emitProgramPreamble = true;
    bool emitProgramEnd = true;
    bool useSafeZBeforeRapid = true;
    double safeZ = 50.0;
    double aNeutralDeg = 0.0;
    double aNeutralToleranceDeg = 0.6;
    double fixedASwitchThresholdDeg = 0.5;
    double connectionTolerance = 0.05;
    double aJoinToleranceDeg = 0.5;
    double feedTolerance = 1.0;
    double powerTolerance = 1.0;
    bool allowLaserOffNoLiftTransition = true;
};
