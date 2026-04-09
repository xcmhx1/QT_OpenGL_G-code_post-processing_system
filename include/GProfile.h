#pragma once

#include <QColor>
#include <QJsonObject>
#include <QMap>
#include <QString>

struct GProfileCodeBlock
{
    QString header;
    QString footer;
    QString comment;

    QJsonObject toJson() const;
    static GProfileCodeBlock fromJson(const QJsonObject& object);
};

class GProfile
{
public:
    GProfile() = default;

    static GProfile createDefaultLaserProfile();
    static GProfile loadFromFile(const QString& filePath, QString* errorMessage = nullptr);

    bool saveToFile(const QString& filePath, QString* errorMessage = nullptr) const;

    void clear();

    void setProfileName(const QString& profileName);
    const QString& profileName() const;

    void setFileCode(const GProfileCodeBlock& codeBlock);
    const GProfileCodeBlock& fileCode() const;

    void setEntityTypeCode(const QString& entityType, const GProfileCodeBlock& codeBlock);
    bool containsEntityTypeCode(const QString& entityType) const;
    GProfileCodeBlock entityTypeCode(const QString& entityType) const;
    void removeEntityTypeCode(const QString& entityType);
    const QMap<QString, GProfileCodeBlock>& entityTypeCodes() const;

    void setLayerCode(const QString& layerName, const GProfileCodeBlock& codeBlock);
    bool containsLayerCode(const QString& layerName) const;
    GProfileCodeBlock layerCode(const QString& layerName) const;
    void removeLayerCode(const QString& layerName);
    const QMap<QString, GProfileCodeBlock>& layerCodes() const;

    void setEntityColorCode(const QString& colorKey, const GProfileCodeBlock& codeBlock);
    void setEntityColorCode(const QColor& color, const GProfileCodeBlock& codeBlock);
    bool containsEntityColorCode(const QString& colorKey) const;
    bool containsEntityColorCode(const QColor& color) const;
    GProfileCodeBlock entityColorCode(const QString& colorKey) const;
    GProfileCodeBlock entityColorCode(const QColor& color) const;
    void removeEntityColorCode(const QString& colorKey);
    void removeEntityColorCode(const QColor& color);
    const QMap<QString, GProfileCodeBlock>& entityColorCodes() const;

    static QString normalizeEntityTypeKey(const QString& entityType);
    static QString normalizeLayerKey(const QString& layerName);
    static QString normalizeColorKey(const QString& colorKey);
    static QString colorKeyFromColor(const QColor& color);
    static QString colorKeyFromAci(int colorIndex);

private:
    QString m_profileName;
    GProfileCodeBlock m_fileCode;
    QMap<QString, GProfileCodeBlock> m_entityTypeCodes;
    QMap<QString, GProfileCodeBlock> m_layerCodes;
    QMap<QString, GProfileCodeBlock> m_entityColorCodes;
};
