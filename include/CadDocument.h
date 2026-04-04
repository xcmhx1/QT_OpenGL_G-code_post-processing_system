#pragma once

#include <memory>

#include <QList>
#include <QObject>
#include <QString>

class CadItem;
class DRW_Entity;
class dx_data;

class CadDocument : public QObject
{
public:
    explicit CadDocument(QObject* parent = nullptr);
    ~CadDocument() override;

    void readDxfDocument(const QString& filePath);
    void saveDxfDocument(const QString& filePath);
    void eportDxfDocument(const QString& filePath);

    void clearAll();
    void init();
    CadItem* createCadItem(DRW_Entity* entity);

    std::unique_ptr<dx_data> m_data;
    QList<CadItem*> m_entities;
};
