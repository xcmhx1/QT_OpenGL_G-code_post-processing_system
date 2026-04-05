#pragma once

#include <memory>
#include <QList>
#include <vector>
#include <QObject>

class CadItem;
class DRW_Entity;
class dx_data;

// Cad文档操作类
class CadDocument : public QObject
{
    Q_OBJECT
public:
    explicit CadDocument(QObject* parent = nullptr);
    ~CadDocument() override;

    // 读取Dxf文档
    void readDxfDocument(const QString& filePath);
    // 保存Dxf文档
    void saveDxfDocument(const QString& filePath);
    // 另存为Dxf文档
    void eportDxfDocument(const QString& filePath);
    // 清空数据
    void clearAll();
    // 初始化
    void init();
    // 原始文档数据
    std::unique_ptr<dx_data> m_data;
    // 内置实体数据数组
    std::vector<std::unique_ptr<CadItem>> m_entities;
};
