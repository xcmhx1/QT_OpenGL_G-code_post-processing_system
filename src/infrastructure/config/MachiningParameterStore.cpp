#include "platform/pch.h"

#include "infrastructure/config/MachiningParameterStore.h"

#include <QSettings>
#include <QVariant>

#include <cmath>
#include <limits>

namespace
{
    constexpr const char* kSettingsOrganization = "GCodePostProcessingSystem";
    constexpr const char* kSettingsApplication = "GCodePostProcessingSystem";
    constexpr const char* kRootGroup = "MachiningParameters";
    constexpr const char* kToolTransferGroup = "ToolTransfer";
    constexpr const char* kRotaryAxisGroup = "RotaryAxis";

    constexpr double kMaximumMachineValue =
        std::numeric_limits<double>::max();

    double readFiniteDouble
    (
        const QSettings& settings,
        const QString& key,
        double fallback,
        double minimum = -kMaximumMachineValue,
        double maximum = kMaximumMachineValue
    )
    {
        if (!settings.contains(key))
        {
            return fallback;
        }

        bool converted = false;
        const double value = settings.value(key).toDouble(&converted);
        return converted && std::isfinite(value)
            && value >= minimum && value <= maximum
            ? value
            : fallback;
    }

    bool readBool
    (
        const QSettings& settings,
        const QString& key,
        bool fallback
    )
    {
        if (!settings.contains(key))
        {
            return fallback;
        }

        const QString value =
            settings.value(key).toString().trimmed().toLower();
        if (value == QStringLiteral("true") || value == QStringLiteral("1"))
        {
            return true;
        }
        if (value == QStringLiteral("false") || value == QStringLiteral("0"))
        {
            return false;
        }
        return fallback;
    }

    template<typename Enum>
    Enum readEnum
    (
        const QSettings& settings,
        const QString& key,
        Enum fallback,
        Enum first,
        Enum second
    )
    {
        if (!settings.contains(key))
        {
            return fallback;
        }

        bool converted = false;
        const int value = settings.value(key).toInt(&converted);
        if (!converted)
        {
            return fallback;
        }

        const Enum candidate = static_cast<Enum>(value);
        return candidate == first || candidate == second
            ? candidate
            : fallback;
    }
}

void MachiningParameterStore::applyTo(GProfile& profile)
{
    QSettings settings
    (
        QString::fromLatin1(kSettingsOrganization),
        QString::fromLatin1(kSettingsApplication)
    );
    settings.beginGroup(QString::fromLatin1(kRootGroup));

    GProfileToolTransferConfig transfer = profile.toolTransferConfig();
    settings.beginGroup(QString::fromLatin1(kToolTransferGroup));
    transfer.rotationSafetyClearance = readFiniteDouble
    (
        settings,
        QStringLiteral("rotationSafetyClearance"),
        transfer.rotationSafetyClearance,
        0.001,
        1000000.0
    );
    transfer.sameZoneTransferClearance = readFiniteDouble
    (
        settings,
        QStringLiteral("sameZoneTransferClearance"),
        transfer.sameZoneTransferClearance,
        0.0,
        1000000.0
    );
    transfer.coordinatedTransferEnabled = readBool
    (
        settings,
        QStringLiteral("coordinatedTransferEnabled"),
        transfer.coordinatedTransferEnabled
    );
    settings.endGroup();

    GProfileRotaryAxisConfig rotary = profile.rotaryAxisConfig();
    settings.beginGroup(QString::fromLatin1(kRotaryAxisGroup));
    rotary.centerY = readFiniteDouble
        (settings, QStringLiteral("centerY"), rotary.centerY);
    rotary.centerZ = readFiniteDouble
        (settings, QStringLiteral("centerZ"), rotary.centerZ);
    rotary.aAxisOffsetDegrees = readFiniteDouble
        (settings, QStringLiteral("aAxisOffsetDegrees"),
            rotary.aAxisOffsetDegrees);
    rotary.machiningPlaneZOffset = readFiniteDouble
    (
        settings,
        QStringLiteral("machiningPlaneZOffset"),
        rotary.machiningPlaneZOffset,
        -1000000.0,
        1000000.0
    );
    rotary.overcutDistance = readFiniteDouble
    (
        settings,
        QStringLiteral("overcutDistance"),
        rotary.overcutDistance,
        0.0,
        100.0
    );
    rotary.lazyRotationProcessing = readBool
        (settings, QStringLiteral("lazyRotationProcessing"),
            rotary.lazyRotationProcessing);
    rotary.invertAAxisDirection = readBool
        (settings, QStringLiteral("invertAAxisDirection"),
            rotary.invertAAxisDirection);
    rotary.keepContinuousAngle = readBool
        (settings, QStringLiteral("keepContinuousAngle"),
            rotary.keepContinuousAngle);
    rotary.useSafeZBeforeRapid = readBool
        (settings, QStringLiteral("useSafeZBeforeRapid"),
            rotary.useSafeZBeforeRapid);
    rotary.useInitialMachinePoint = readBool
        (settings, QStringLiteral("useInitialMachinePoint"),
            rotary.useInitialMachinePoint);
    rotary.initialMachineX = readFiniteDouble
        (settings, QStringLiteral("initialMachineX"),
            rotary.initialMachineX);
    rotary.initialMachineY = readFiniteDouble
        (settings, QStringLiteral("initialMachineY"),
            rotary.initialMachineY);
    rotary.initialMachineZ = readFiniteDouble
        (settings, QStringLiteral("initialMachineZ"),
            rotary.initialMachineZ);
    rotary.perimeterSweepDirection =
        readEnum<GProfilePerimeterSweepDirection>
        (
            settings,
            QStringLiteral("perimeterSweepDirection"),
            rotary.perimeterSweepDirection,
            GProfilePerimeterSweepDirection::Clockwise,
            GProfilePerimeterSweepDirection::CounterClockwise
        );
    rotary.longitudinalSweepDirection =
        readEnum<GProfileLongitudinalSweepDirection>
        (
            settings,
            QStringLiteral("longitudinalSweepDirection"),
            rotary.longitudinalSweepDirection,
            GProfileLongitudinalSweepDirection::PositiveX,
            GProfileLongitudinalSweepDirection::NegativeX
        );
    settings.endGroup();
    settings.endGroup();

    profile.setToolTransferConfig(transfer);
    profile.setRotaryAxisConfig(rotary);
}

void MachiningParameterStore::save
(
    const GProfileToolTransferConfig& transfer,
    const GProfileRotaryAxisConfig& rotary
)
{
    QSettings settings
    (
        QString::fromLatin1(kSettingsOrganization),
        QString::fromLatin1(kSettingsApplication)
    );
    settings.beginGroup(QString::fromLatin1(kRootGroup));

    settings.beginGroup(QString::fromLatin1(kToolTransferGroup));
    settings.setValue(QStringLiteral("rotationSafetyClearance"),
        transfer.rotationSafetyClearance);
    settings.setValue(QStringLiteral("sameZoneTransferClearance"),
        transfer.sameZoneTransferClearance);
    settings.setValue(QStringLiteral("coordinatedTransferEnabled"),
        transfer.coordinatedTransferEnabled);
    settings.endGroup();

    settings.beginGroup(QString::fromLatin1(kRotaryAxisGroup));
    settings.setValue(QStringLiteral("centerY"), rotary.centerY);
    settings.setValue(QStringLiteral("centerZ"), rotary.centerZ);
    settings.setValue(QStringLiteral("aAxisOffsetDegrees"),
        rotary.aAxisOffsetDegrees);
    settings.setValue(QStringLiteral("machiningPlaneZOffset"),
        rotary.machiningPlaneZOffset);
    settings.setValue(QStringLiteral("overcutDistance"),
        rotary.overcutDistance);
    settings.setValue(QStringLiteral("lazyRotationProcessing"),
        rotary.lazyRotationProcessing);
    settings.setValue(QStringLiteral("invertAAxisDirection"),
        rotary.invertAAxisDirection);
    settings.setValue(QStringLiteral("keepContinuousAngle"),
        rotary.keepContinuousAngle);
    settings.setValue(QStringLiteral("useSafeZBeforeRapid"),
        rotary.useSafeZBeforeRapid);
    settings.setValue(QStringLiteral("useInitialMachinePoint"),
        rotary.useInitialMachinePoint);
    settings.setValue(QStringLiteral("initialMachineX"),
        rotary.initialMachineX);
    settings.setValue(QStringLiteral("initialMachineY"),
        rotary.initialMachineY);
    settings.setValue(QStringLiteral("initialMachineZ"),
        rotary.initialMachineZ);
    settings.setValue(QStringLiteral("perimeterSweepDirection"),
        static_cast<int>(rotary.perimeterSweepDirection));
    settings.setValue(QStringLiteral("longitudinalSweepDirection"),
        static_cast<int>(rotary.longitudinalSweepDirection));
    settings.endGroup();
    settings.endGroup();
    settings.sync();
}
