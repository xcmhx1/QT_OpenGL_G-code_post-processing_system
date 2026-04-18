#include "pch.h"

#include "GCodeGenerator4Axis.h"

#include <cmath>
#include <limits>

namespace
{
    bool almostEqual(double left, double right, double eps = 1.0e-9)
    {
        return std::abs(left - right) <= eps;
    }

    bool isFinite(double value)
    {
        return std::isfinite(value);
    }

    void appendAxisWord
    (
        QString& line,
        QChar axis,
        double value,
        int precision
    )
    {
        line.append(QStringLiteral(" %1%2").arg(axis, QString::number(value, 'f', precision)));
    }

    ToolpathPoint4Axis buildSafeZPoint
    (
        const ToolpathPoint4Axis& reference,
        const MachineConfig4Axis& machine
    )
    {
        ToolpathPoint4Axis safePoint = reference;
        safePoint.machinePos.setZ(static_cast<float>(machine.safeZ));
        return safePoint;
    }
}

QString GCodeGenerator4Axis::formatNumber(double value, int precision)
{
    return QString::number(value, 'f', precision);
}

void GCodeGenerator4Axis::emitLaserOn(QStringList& lines, const MachineConfig4Axis& machine, bool& laserOn)
{
    if (laserOn)
    {
        return;
    }

    lines.append(machine.laserOnCode);
    laserOn = true;
}

void GCodeGenerator4Axis::emitLaserOff(QStringList& lines, const MachineConfig4Axis& machine, bool& laserOn)
{
    if (!laserOn)
    {
        return;
    }

    lines.append(machine.laserOffCode);
    laserOn = false;
}

bool GCodeGenerator4Axis::emitRapid
(
    QStringList& lines,
    const ToolpathPoint4Axis& point,
    const MachineConfig4Axis& machine,
    bool includeA
)
{
    QString line = machine.rapidCode;
    bool hasAxis = false;

    if (isFinite(point.machinePos.x()))
    {
        appendAxisWord(line, QChar('X'), point.machinePos.x(), machine.coordPrecision);
        hasAxis = true;
    }

    if (isFinite(point.machinePos.y()))
    {
        appendAxisWord(line, QChar('Y'), point.machinePos.y(), machine.coordPrecision);
        hasAxis = true;
    }

    if (isFinite(point.machinePos.z()))
    {
        appendAxisWord(line, QChar('Z'), point.machinePos.z(), machine.coordPrecision);
        hasAxis = true;
    }

    if (includeA && isFinite(point.aDeg))
    {
        appendAxisWord(line, QChar('A'), point.aDeg, machine.anglePrecision);
        hasAxis = true;
    }

    if (!hasAxis)
    {
        return false;
    }

    lines.append(line);
    return true;
}

bool GCodeGenerator4Axis::emitLinear4Axis
(
    QStringList& lines,
    const ToolpathPoint4Axis& point,
    const MachineConfig4Axis& machine,
    bool includeA,
    bool includeFeed
)
{
    QString line = machine.linearCode;
    bool hasWord = false;

    if (isFinite(point.machinePos.x()))
    {
        appendAxisWord(line, QChar('X'), point.machinePos.x(), machine.coordPrecision);
        hasWord = true;
    }

    if (isFinite(point.machinePos.y()))
    {
        appendAxisWord(line, QChar('Y'), point.machinePos.y(), machine.coordPrecision);
        hasWord = true;
    }

    if (isFinite(point.machinePos.z()))
    {
        appendAxisWord(line, QChar('Z'), point.machinePos.z(), machine.coordPrecision);
        hasWord = true;
    }

    if (includeA && isFinite(point.aDeg))
    {
        appendAxisWord(line, QChar('A'), point.aDeg, machine.anglePrecision);
        hasWord = true;
    }

    if (includeFeed && isFinite(point.feed))
    {
        appendAxisWord(line, QChar('F'), point.feed, machine.feedPrecision);
        hasWord = true;
    }

    if (!hasWord)
    {
        return false;
    }

    lines.append(line);
    return true;
}

QString GCodeGenerator4Axis::generate
(
    const QVector<ToolpathSegment4Axis>& segments,
    const RotaryConfig& rotary,
    const MachineConfig4Axis& machine,
    QStringList* warnings
) const
{
    Q_UNUSED(rotary);

    QStringList lines;
    lines.reserve(512);

    if (machine.emitProgramPreamble)
    {
        lines.append(QStringLiteral("%"));
        lines.append(machine.absoluteMode ? QStringLiteral("G90") : QStringLiteral("G91"));
        lines.append(machine.metricUnits ? QStringLiteral("G21") : QStringLiteral("G20"));
    }

    bool laserOn = false;
    bool hasCurrentPoint = false;
    ToolpathPoint4Axis currentPoint;
    const float qnan = std::numeric_limits<float>::quiet_NaN();
    const double dnan = std::numeric_limits<double>::quiet_NaN();

    for (int segmentIndex = 0; segmentIndex < segments.size(); ++segmentIndex)
    {
        const ToolpathSegment4Axis& segment = segments.at(segmentIndex);

        if (segment.points.size() < 2)
        {
            continue;
        }

        const ToolpathPoint4Axis& startPoint = segment.points.front();
        const ToolpathPoint4Axis& endPoint = segment.points.back();
        const bool isFixedASegment = segment.mode == SegmentMotionMode::AFixed;

        emitLaserOff(lines, machine, laserOn);

        const bool needSafeSwitch = hasCurrentPoint
            && isFixedASegment
            && std::abs(startPoint.aDeg - currentPoint.aDeg) > machine.fixedASwitchThresholdDeg;

        if (hasCurrentPoint && isFixedASegment)
        {
            const double deltaA = std::abs(startPoint.aDeg - currentPoint.aDeg);

            if (deltaA > machine.fixedASwitchThresholdDeg && machine.useSafeZBeforeRapid)
            {
                ToolpathPoint4Axis safeCurrent = buildSafeZPoint(currentPoint, machine);
                if (emitRapid(lines, safeCurrent, machine, true))
                {
                    currentPoint = safeCurrent;
                }

                ToolpathPoint4Axis safeTarget = buildSafeZPoint(startPoint, machine);
                if (emitRapid(lines, safeTarget, machine, true))
                {
                    currentPoint = safeTarget;
                }

                if (emitRapid(lines, startPoint, machine, true))
                {
                    currentPoint = startPoint;
                }
            }
            else
            {
                ToolpathPoint4Axis rapidPoint = startPoint;

                if (almostEqual(rapidPoint.machinePos.x(), currentPoint.machinePos.x()))
                {
                    rapidPoint.machinePos.setX(qnan);
                }

                if (almostEqual(rapidPoint.machinePos.y(), currentPoint.machinePos.y()))
                {
                    rapidPoint.machinePos.setY(qnan);
                }

                if (almostEqual(rapidPoint.machinePos.z(), currentPoint.machinePos.z()))
                {
                    rapidPoint.machinePos.setZ(qnan);
                }

                if (almostEqual(rapidPoint.aDeg, currentPoint.aDeg))
                {
                    rapidPoint.aDeg = dnan;
                }

                if (emitRapid(lines, rapidPoint, machine, true))
                {
                    currentPoint = startPoint;
                }
            }
        }
        else
        {
            if (hasCurrentPoint
                && machine.useSafeZBeforeRapid
                && (!almostEqual(currentPoint.machinePos.x(), startPoint.machinePos.x())
                    || !almostEqual(currentPoint.machinePos.y(), startPoint.machinePos.y())
                    || !almostEqual(currentPoint.machinePos.z(), startPoint.machinePos.z())
                    || !almostEqual(currentPoint.aDeg, startPoint.aDeg)))
            {
                ToolpathPoint4Axis safeCurrent = buildSafeZPoint(currentPoint, machine);
                if (emitRapid(lines, safeCurrent, machine, true))
                {
                    currentPoint = safeCurrent;
                }
            }

            ToolpathPoint4Axis rapidPoint = startPoint;

            if (hasCurrentPoint)
            {
                if (almostEqual(rapidPoint.machinePos.x(), currentPoint.machinePos.x()))
                {
                    rapidPoint.machinePos.setX(qnan);
                }

                if (almostEqual(rapidPoint.machinePos.y(), currentPoint.machinePos.y()))
                {
                    rapidPoint.machinePos.setY(qnan);
                }

                if (almostEqual(rapidPoint.machinePos.z(), currentPoint.machinePos.z()))
                {
                    rapidPoint.machinePos.setZ(qnan);
                }

                if (almostEqual(rapidPoint.aDeg, currentPoint.aDeg))
                {
                    rapidPoint.aDeg = dnan;
                }
            }

            if (emitRapid(lines, rapidPoint, machine, true))
            {
                currentPoint = startPoint;
            }
        }

        emitLaserOn(lines, machine, laserOn);

        bool feedEmitted = false;

        for (int pointIndex = 1; pointIndex < segment.points.size(); ++pointIndex)
        {
            const ToolpathPoint4Axis& point = segment.points.at(pointIndex);
            const bool includeA = !isFixedASegment || pointIndex == 1;
            ToolpathPoint4Axis linearPoint = point;

            if (hasCurrentPoint)
            {
                if (almostEqual(linearPoint.machinePos.x(), currentPoint.machinePos.x()))
                {
                    linearPoint.machinePos.setX(qnan);
                }

                if (almostEqual(linearPoint.machinePos.y(), currentPoint.machinePos.y()))
                {
                    linearPoint.machinePos.setY(qnan);
                }

                if (almostEqual(linearPoint.machinePos.z(), currentPoint.machinePos.z()))
                {
                    linearPoint.machinePos.setZ(qnan);
                }

                if (!includeA || almostEqual(linearPoint.aDeg, currentPoint.aDeg))
                {
                    linearPoint.aDeg = dnan;
                }
            }

            if (feedEmitted)
            {
                linearPoint.feed = dnan;
            }

            if (emitLinear4Axis(lines, linearPoint, machine, includeA, !feedEmitted))
            {
                currentPoint = point;
                hasCurrentPoint = true;
                feedEmitted = true;
            }
        }

        currentPoint = endPoint;
        hasCurrentPoint = true;

        if (needSafeSwitch && warnings != nullptr)
        {
            warnings->append(QStringLiteral("第 %1 段检测到固定 A 姿态切换，已注入激光关断与安全抬高流程。").arg(segmentIndex + 1));
        }
    }

    emitLaserOff(lines, machine, laserOn);
    if (machine.emitProgramEnd)
    {
        lines.append(QStringLiteral("M30"));
        lines.append(QStringLiteral("%"));
    }

    if (lines.isEmpty())
    {
        return QString();
    }

    return lines.join(QStringLiteral("\r\n")) + QStringLiteral("\r\n");
}
