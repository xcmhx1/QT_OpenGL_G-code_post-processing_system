#include "platform/pch.h"

#include "desktop/AppLicense.h"

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSettings>
#include <QSysInfo>

namespace
{
    constexpr const char* kLicenseSecret = "GCodePostProcessingSystem.LightCommercial.2026";
    constexpr int kSignatureRounds = 4096;

    QString normalizedEdition(const QString& value)
    {
        const QString normalized = value.trimmed().toLower();
        return normalized == QStringLiteral("pro") ? QStringLiteral("pro") : QStringLiteral("lite");
    }

    QString stableMachineSeed()
    {
#ifdef Q_OS_WIN
        QSettings machineGuidSettings
        (
            QStringLiteral("HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Cryptography"),
            QSettings::NativeFormat
        );
        const QString machineGuid = machineGuidSettings.value(QStringLiteral("MachineGuid")).toString().trimmed();

        if (!machineGuid.isEmpty())
        {
            return QStringLiteral("win-machine-guid:%1").arg(machineGuid.toUpper());
        }
#endif

        const QByteArray uniqueId = QSysInfo::machineUniqueId();

        if (!uniqueId.isEmpty())
        {
            return QStringLiteral("qt-machine-id:%1").arg(QString::fromLatin1(uniqueId.toHex()));
        }

        return QStringLiteral("host:%1").arg(QSysInfo::machineHostName().trimmed().toUpper());
    }
}

AppLicense AppLicense::load()
{
    AppLicense license;
    const QString filePath = QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("license.dat"));
    QFile file(filePath);

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        return license;
    }

    const QJsonDocument document = QJsonDocument::fromJson(file.readAll());

    if (!document.isObject())
    {
        license.m_message = QStringLiteral("授权文件格式无效：当前按 Lite 版本运行。");
        return license;
    }

    const QJsonObject object = document.object();
    const QString edition = normalizedEdition(object.value(QStringLiteral("edition")).toString());
    const QString machineId = object.value(QStringLiteral("machineId")).toString().trimmed();
    const QString licenseId = object.value(QStringLiteral("licenseId")).toString().trimmed();
    const QString signature = object.value(QStringLiteral("signature")).toString().trimmed().toLower();

    if (machineId.isEmpty() || licenseId.isEmpty())
    {
        license.m_message = QStringLiteral("授权文件缺少机器码或授权编号：当前按 Lite 版本运行。");
        return license;
    }

    const QString payload = signaturePayload(machineId, licenseId);
    const QString expectedSignature = signatureForPayload(payload);

    if (signature.isEmpty() || signature != expectedSignature)
    {
        license.m_message = QStringLiteral("授权签名无效：当前按 Lite 版本运行。");
        return license;
    }

    if (machineId != currentMachineId())
    {
        license.m_message = QStringLiteral("授权文件不属于当前机器：当前按 Lite 版本运行。");
        return license;
    }

    license.m_valid = true;
    license.m_edition = edition == QStringLiteral("pro") ? AppEdition::Pro : AppEdition::Lite;
    license.m_licenseId = licenseId;
    license.m_message = QStringLiteral("%1 授权").arg(license.editionName());
    return license;
}

QString AppLicense::currentMachineId()
{
    return QString::fromLatin1(QCryptographicHash::hash(stableMachineSeed().toUtf8(), QCryptographicHash::Sha256).toHex()).toUpper();
}

bool AppLicense::isValid() const
{
    return m_valid;
}

bool AppLicense::allows(AppFeature feature) const
{
    Q_UNUSED(feature);
    return m_valid && m_edition == AppEdition::Pro;
}

AppEdition AppLicense::edition() const
{
    return m_edition;
}

QString AppLicense::editionName() const
{
    return m_edition == AppEdition::Pro ? QStringLiteral("Pro") : QStringLiteral("Lite");
}

QString AppLicense::licenseId() const
{
    return m_licenseId;
}

QString AppLicense::statusText() const
{
    return m_message;
}

QString AppLicense::signaturePayload(const QString& machineId, const QString& licenseId)
{
    return QStringLiteral("machineId=%1\nlicenseId=%2")
        .arg(machineId, licenseId);
}

QString AppLicense::signatureForPayload(const QString& payload)
{
    QByteArray digest = (payload + QString::fromLatin1(kLicenseSecret)).toUtf8();

    for (int round = 0; round < kSignatureRounds; ++round)
    {
        digest = QCryptographicHash::hash
        (
            digest + QByteArray::number(round) + QByteArray(kLicenseSecret),
            QCryptographicHash::Sha256
        );
    }

    return QString::fromLatin1(digest.toHex());
}
