#include "pch.h"

#include "GProfile.h"

#include <QFile>
#include <QJsonDocument>
#include <QJsonValue>

namespace
{
    constexpr const char* kProfileNameKey = "profileName";
    constexpr const char* kFileCodeKey = "fileCode";
    constexpr const char* kEntityTypeCodesKey = "entityTypeCodes";
    constexpr const char* kLayerCodesKey = "layerCodes";
    constexpr const char* kEntityColorCodesKey = "entityColorCodes";
    constexpr const char* kHeaderKey = "header";
    constexpr const char* kFooterKey = "footer";
    constexpr const char* kCommentKey = "comment";
}

QJsonObject GProfileCodeBlock::toJson() const
{
    QJsonObject object;
    object.insert(kHeaderKey, header);
    object.insert(kFooterKey, footer);
    object.insert(kCommentKey, comment);
    return object;
}

GProfileCodeBlock GProfileCodeBlock::fromJson(const QJsonObject& object)
{
    GProfileCodeBlock codeBlock;
    codeBlock.header = object.value(kHeaderKey).toString();
    codeBlock.footer = object.value(kFooterKey).toString();
    codeBlock.comment = object.value(kCommentKey).toString();
    return codeBlock;
}

GProfile GProfile::createDefaultLaserProfile()
{
    GProfile profile;
    profile.setProfileName(QStringLiteral("默认激光二维配置"));
    profile.setFileCode
    (
        {
            QStringLiteral("%\nG90\nG17\nG21\nG54"),
            QStringLiteral("M05\nG00 X0.000 Y0.000\nM30\n%"),
            QStringLiteral("文件整体头尾")
        }
    );

    const GProfileCodeBlock cuttingCode
    {
        QStringLiteral("M03"),
        QStringLiteral("M05"),
        QStringLiteral("实体类型默认加工头尾")
    };

    profile.setEntityTypeCode(QStringLiteral("LINE"), cuttingCode);
    profile.setEntityTypeCode(QStringLiteral("ARC"), cuttingCode);
    profile.setEntityTypeCode(QStringLiteral("CIRCLE"), cuttingCode);
    profile.setEntityTypeCode(QStringLiteral("ELLIPSE"), cuttingCode);
    profile.setEntityTypeCode(QStringLiteral("POLYLINE"), cuttingCode);
    profile.setEntityTypeCode(QStringLiteral("LWPOLYLINE"), cuttingCode);
    profile.setEntityTypeCode
    (
        QStringLiteral("POINT"),
        {
            QString(),
            QString(),
            QStringLiteral("点图元默认不出加工启停指令")
        }
    );

    profile.setEntityColorCode
    (
        QStringLiteral("ACI:1"),
        {
            QStringLiteral("M03"),
            QStringLiteral("M05"),
            QStringLiteral("红色实体加工头尾示例")
        }
    );

    return profile;
}

GProfile GProfile::loadFromFile(const QString& filePath, QString* errorMessage)
{
    GProfile profile;

    QFile file(filePath);

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("无法打开配置文件: %1").arg(filePath);
        }

        return profile;
    }

    const QByteArray payload = file.readAll();
    file.close();

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(payload, &parseError);

    if (parseError.error != QJsonParseError::NoError || !document.isObject())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("配置文件 JSON 解析失败: %1").arg(parseError.errorString());
        }

        return GProfile();
    }

    const QJsonObject rootObject = document.object();
    profile.m_profileName = rootObject.value(kProfileNameKey).toString();
    profile.m_fileCode = GProfileCodeBlock::fromJson(rootObject.value(kFileCodeKey).toObject());

    const QJsonObject entityTypeObject = rootObject.value(kEntityTypeCodesKey).toObject();

    for (auto it = entityTypeObject.begin(); it != entityTypeObject.end(); ++it)
    {
        profile.m_entityTypeCodes.insert(normalizeEntityTypeKey(it.key()), GProfileCodeBlock::fromJson(it.value().toObject()));
    }

    const QJsonObject layerObject = rootObject.value(kLayerCodesKey).toObject();

    for (auto it = layerObject.begin(); it != layerObject.end(); ++it)
    {
        profile.m_layerCodes.insert(normalizeLayerKey(it.key()), GProfileCodeBlock::fromJson(it.value().toObject()));
    }

    const QJsonObject entityColorObject = rootObject.value(kEntityColorCodesKey).toObject();

    for (auto it = entityColorObject.begin(); it != entityColorObject.end(); ++it)
    {
        profile.m_entityColorCodes.insert(normalizeColorKey(it.key()), GProfileCodeBlock::fromJson(it.value().toObject()));
    }

    if (errorMessage != nullptr)
    {
        errorMessage->clear();
    }

    return profile;
}

bool GProfile::saveToFile(const QString& filePath, QString* errorMessage) const
{
    QJsonObject rootObject;
    rootObject.insert(kProfileNameKey, m_profileName);
    rootObject.insert(kFileCodeKey, m_fileCode.toJson());

    QJsonObject entityTypeObject;

    for (auto it = m_entityTypeCodes.cbegin(); it != m_entityTypeCodes.cend(); ++it)
    {
        entityTypeObject.insert(it.key(), it.value().toJson());
    }

    rootObject.insert(kEntityTypeCodesKey, entityTypeObject);

    QJsonObject layerObject;

    for (auto it = m_layerCodes.cbegin(); it != m_layerCodes.cend(); ++it)
    {
        layerObject.insert(it.key(), it.value().toJson());
    }

    rootObject.insert(kLayerCodesKey, layerObject);

    QJsonObject entityColorObject;

    for (auto it = m_entityColorCodes.cbegin(); it != m_entityColorCodes.cend(); ++it)
    {
        entityColorObject.insert(it.key(), it.value().toJson());
    }

    rootObject.insert(kEntityColorCodesKey, entityColorObject);

    QFile file(filePath);

    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("无法写入配置文件: %1").arg(filePath);
        }

        return false;
    }

    const QJsonDocument document(rootObject);

    if (file.write(document.toJson(QJsonDocument::Indented)) < 0)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("写入配置文件失败: %1").arg(filePath);
        }

        return false;
    }

    file.close();

    if (errorMessage != nullptr)
    {
        errorMessage->clear();
    }

    return true;
}

void GProfile::clear()
{
    m_profileName.clear();
    m_fileCode = GProfileCodeBlock();
    m_entityTypeCodes.clear();
    m_layerCodes.clear();
    m_entityColorCodes.clear();
}

void GProfile::setProfileName(const QString& profileName)
{
    m_profileName = profileName.trimmed();
}

const QString& GProfile::profileName() const
{
    return m_profileName;
}

void GProfile::setFileCode(const GProfileCodeBlock& codeBlock)
{
    m_fileCode = codeBlock;
}

const GProfileCodeBlock& GProfile::fileCode() const
{
    return m_fileCode;
}

void GProfile::setEntityTypeCode(const QString& entityType, const GProfileCodeBlock& codeBlock)
{
    const QString normalizedKey = normalizeEntityTypeKey(entityType);

    if (normalizedKey.isEmpty())
    {
        return;
    }

    m_entityTypeCodes.insert(normalizedKey, codeBlock);
}

bool GProfile::containsEntityTypeCode(const QString& entityType) const
{
    return m_entityTypeCodes.contains(normalizeEntityTypeKey(entityType));
}

GProfileCodeBlock GProfile::entityTypeCode(const QString& entityType) const
{
    return m_entityTypeCodes.value(normalizeEntityTypeKey(entityType));
}

void GProfile::removeEntityTypeCode(const QString& entityType)
{
    m_entityTypeCodes.remove(normalizeEntityTypeKey(entityType));
}

const QMap<QString, GProfileCodeBlock>& GProfile::entityTypeCodes() const
{
    return m_entityTypeCodes;
}

void GProfile::setLayerCode(const QString& layerName, const GProfileCodeBlock& codeBlock)
{
    const QString normalizedKey = normalizeLayerKey(layerName);

    if (normalizedKey.isEmpty())
    {
        return;
    }

    m_layerCodes.insert(normalizedKey, codeBlock);
}

bool GProfile::containsLayerCode(const QString& layerName) const
{
    return m_layerCodes.contains(normalizeLayerKey(layerName));
}

GProfileCodeBlock GProfile::layerCode(const QString& layerName) const
{
    return m_layerCodes.value(normalizeLayerKey(layerName));
}

void GProfile::removeLayerCode(const QString& layerName)
{
    m_layerCodes.remove(normalizeLayerKey(layerName));
}

const QMap<QString, GProfileCodeBlock>& GProfile::layerCodes() const
{
    return m_layerCodes;
}

void GProfile::setEntityColorCode(const QString& colorKey, const GProfileCodeBlock& codeBlock)
{
    const QString normalizedKey = normalizeColorKey(colorKey);

    if (normalizedKey.isEmpty())
    {
        return;
    }

    m_entityColorCodes.insert(normalizedKey, codeBlock);
}

void GProfile::setEntityColorCode(const QColor& color, const GProfileCodeBlock& codeBlock)
{
    setEntityColorCode(colorKeyFromColor(color), codeBlock);
}

bool GProfile::containsEntityColorCode(const QString& colorKey) const
{
    return m_entityColorCodes.contains(normalizeColorKey(colorKey));
}

bool GProfile::containsEntityColorCode(const QColor& color) const
{
    return containsEntityColorCode(colorKeyFromColor(color));
}

GProfileCodeBlock GProfile::entityColorCode(const QString& colorKey) const
{
    return m_entityColorCodes.value(normalizeColorKey(colorKey));
}

GProfileCodeBlock GProfile::entityColorCode(const QColor& color) const
{
    return entityColorCode(colorKeyFromColor(color));
}

void GProfile::removeEntityColorCode(const QString& colorKey)
{
    m_entityColorCodes.remove(normalizeColorKey(colorKey));
}

void GProfile::removeEntityColorCode(const QColor& color)
{
    removeEntityColorCode(colorKeyFromColor(color));
}

const QMap<QString, GProfileCodeBlock>& GProfile::entityColorCodes() const
{
    return m_entityColorCodes;
}

QString GProfile::normalizeEntityTypeKey(const QString& entityType)
{
    return entityType.trimmed().toUpper();
}

QString GProfile::normalizeLayerKey(const QString& layerName)
{
    return layerName.trimmed();
}

QString GProfile::normalizeColorKey(const QString& colorKey)
{
    QString normalizedKey = colorKey.trimmed().toUpper();

    if (normalizedKey.isEmpty())
    {
        return QString();
    }

    if (normalizedKey == QStringLiteral("BYLAYER") || normalizedKey == QStringLiteral("LAYER"))
    {
        return QStringLiteral("BYLAYER");
    }

    if (normalizedKey == QStringLiteral("BYBLOCK") || normalizedKey == QStringLiteral("BLOCK"))
    {
        return QStringLiteral("BYBLOCK");
    }

    if (normalizedKey.startsWith(QStringLiteral("ACI:"))
        || normalizedKey.startsWith(QStringLiteral("ACI-"))
        || normalizedKey.startsWith(QStringLiteral("ACI_")))
    {
        QString numberText = normalizedKey.mid(4).trimmed();

        if (numberText.startsWith(':') || numberText.startsWith('-') || numberText.startsWith('_'))
        {
            numberText.remove(0, 1);
        }

        bool ok = false;
        const int colorIndex = numberText.toInt(&ok);
        return ok ? colorKeyFromAci(colorIndex) : QString();
    }

    bool pureNumberOk = false;
    const int pureNumber = normalizedKey.toInt(&pureNumberOk);

    if (pureNumberOk)
    {
        return colorKeyFromAci(pureNumber);
    }

    if (!normalizedKey.startsWith('#'))
    {
        normalizedKey.prepend('#');
    }

    return normalizedKey;
}

QString GProfile::colorKeyFromColor(const QColor& color)
{
    if (!color.isValid())
    {
        return QString();
    }

    return color.name(QColor::HexRgb).toUpper();
}

QString GProfile::colorKeyFromAci(int colorIndex)
{
    return colorIndex >= 0 ? QStringLiteral("ACI:%1").arg(colorIndex) : QString();
}
