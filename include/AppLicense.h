#pragma once

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
    bool allows(AppFeature feature) const;
    AppEdition edition() const;
    QString editionName() const;
    QString licenseId() const;
    QString statusText() const;

private:
    static QString signaturePayload(const QString& machineId, const QString& licenseId);
    static QString signatureForPayload(const QString& payload);

private:
    bool m_valid = false;
    AppEdition m_edition = AppEdition::Lite;
    QString m_licenseId;
    QString m_message = QStringLiteral("未授权：当前按 Lite 版本运行。");
};
