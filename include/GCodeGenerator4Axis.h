#pragma once

#include "Rotary4AxisTypes.h"

#include <QString>
#include <QStringList>

class GCodeGenerator4Axis
{
public:
    QString generate
    (
        const QVector<ToolpathSegment4Axis>& segments,
        const RotaryConfig& rotary,
        const MachineConfig4Axis& machine,
        QStringList* warnings = nullptr
    ) const;

private:
    static QString formatNumber(double value, int precision);
    static void emitLaserOn(QStringList& lines, const MachineConfig4Axis& machine, bool& laserOn);
    static void emitLaserOff(QStringList& lines, const MachineConfig4Axis& machine, bool& laserOn);
    static bool emitRapid
    (
        QStringList& lines,
        const ToolpathPoint4Axis& point,
        const MachineConfig4Axis& machine,
        bool includeA
    );
    static bool emitLinear4Axis
    (
        QStringList& lines,
        const ToolpathPoint4Axis& point,
        const MachineConfig4Axis& machine,
        bool includeA,
        bool includeFeed
    );
};
