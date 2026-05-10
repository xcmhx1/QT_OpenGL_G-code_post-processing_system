#include "pch.h"

#include "AppLicense.h"

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDate>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSysInfo>

namespace
{
    constexpr const char* kLicenseSecret = "GCodePostProcessingSystem.LightCommercial.2026";

    QString normalizedEdition(const QString& value)
    {
        const QString normalized = value.trimmed().toLower();
        return normalized == QStringLiteral("pro") ? QStringLiteral("pro") : QStringLiteral("lite");
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
    const QString customer = object.value(QStringLiteral("customer")).toString().trimmed();
    const QString edition = normalizedEdition(object.value(QStringLiteral("edition")).toString());
    const QString expires = object.value(QStringLiteral("expires")).toString().trimmed();
    const QString machineId = object.value(QStringLiteral("machineId")).toString().trimmed();
    const QString licenseId = object.value(QStringLiteral("licenseId")).toString().trimmed();
    const QString signature = object.value(QStringLiteral("signature")).toString().trimmed().toLower();

    const QString payload = signaturePayload(customer, edition, expires, machineId, licenseId);
    const QString expectedSignature = signatureForPayload(payload);

    if (signature.isEmpty() || signature != expectedSignature)
    {
        license.m_message = QStringLiteral("授权签名无效：当前按 Lite 版本运行。");
        return license;
    }

    if (!machineId.isEmpty() && machineId != currentMachineId())
    {
        license.m_message = QStringLiteral("授权文件不属于当前机器：当前按 Lite 版本运行。");
        return license;
    }

    const QDate expiresOn = QDate::fromString(expires, Qt::ISODate);

    if (!expires.isEmpty() && (!expiresOn.isValid() || expiresOn < QDate::currentDate()))
    {
        license.m_expired = true;
        license.m_expiresOn = expiresOn;
        license.m_message = QStringLiteral("授权已过期：当前按 Lite 版本运行。");
        return license;
    }

    license.m_valid = true;
    license.m_edition = edition == QStringLiteral("pro") ? AppEdition::Pro : AppEdition::Lite;
    license.m_customerName = customer;
    license.m_licenseId = licenseId;
    license.m_expiresOn = expiresOn;
    license.m_message = QStringLiteral("%1 授权%2%3")
        .arg(license.editionName())
        .arg(customer.isEmpty() ? QString() : QStringLiteral(" - %1").arg(customer))
        .arg(expiresOn.isValid() ? QStringLiteral("，有效期至 %1").arg(expiresOn.toString(Qt::ISODate)) : QString());
    return license;
}

QString AppLicense::currentMachineId()
{
    const QByteArray machineId = QSysInfo::machineUniqueId();

    if (!machineId.isEmpty())
    {
        return QString::fromLatin1(machineId.toHex());
    }

    return QString::fromLatin1(QCryptographicHash::hash(QSysInfo::machineHostName().toUtf8(), QCryptographicHash::Sha256).toHex());
}

bool AppLicense::isValid() const
{
    return m_valid;
}

bool AppLicense::isExpired() const
{
    return m_expired;
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

QString AppLicense::customerName() const
{
    return m_customerName;
}

QString AppLicense::licenseId() const
{
    return m_licenseId;
}

QDate AppLicense::expiresOn() const
{
    return m_expiresOn;
}

QString AppLicense::statusText() const
{
    return m_message;
}

QString AppLicense::signaturePayload(const QString& customer, const QString& edition, const QString& expires, const QString& machineId, const QString& licenseId)
{
    return QStringLiteral("customer=%1\nedition=%2\nexpires=%3\nmachineId=%4\nlicenseId=%5")
        .arg(customer, edition, expires, machineId, licenseId);
}

QString AppLicense::signatureForPayload(const QString& payload)
{
    return QString::fromLatin1(QCryptographicHash::hash((payload + QString::fromLatin1(kLicenseSecret)).toUtf8(), QCryptographicHash::Sha256).toHex());
}
