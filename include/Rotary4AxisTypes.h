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
    double maxDeltaADegPerStep = 5.0;
    double maxLinearStep = 1.0;
    double fixedASwitchThresholdDeg = 0.5;
};

struct SamplePoint
{
    QVector3D pos;
    QVector3D normal;
    bool hasNormal = false;
    double targetA = 0.0;
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
    QVector<ToolpathPoint4Axis> points;
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
    double fixedASwitchThresholdDeg = 0.5;
};
