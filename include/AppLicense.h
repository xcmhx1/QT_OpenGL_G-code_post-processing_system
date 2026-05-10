#pragma once

#include <QDate>
#include <QString>

enum class AppEdition
{
    Lite,
    Pro
};

enum class AppFeature
{
    BitmapImport,
    SmartSort,
    FourAxisExport,
    ProfileSettings,
    CustomAppearance
};

class AppLicense
{
public:
    static AppLicense load();
    static QString currentMachineId();

    bool isValid() const;
    bool isExpired() const;
    bool allows(AppFeature feature) const;
    AppEdition edition() const;
    QString editionName() const;
    QString customerName() const;
    QString licenseId() const;
    QDate expiresOn() const;
    QString statusText() const;

private:
    static QString signaturePayload(const QString& customer, const QString& edition, const QString& expires, const QString& machineId, const QString& licenseId);
    static QString signatureForPayload(const QString& payload);

private:
    bool m_valid = false;
    bool m_expired = false;
    AppEdition m_edition = AppEdition::Lite;
    QString m_customerName;
    QString m_licenseId;
    QString m_message = QStringLiteral("未授权：当前按 Lite 版本运行。");
    QDate m_expiresOn;
};
