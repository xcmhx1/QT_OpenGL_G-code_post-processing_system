// 声明 CadDocument 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 文档模型模块，负责持有原始 DXF 数据、内部图元容器以及场景变更通知。
#pragma once

#include <memory>
#include <QList>
#include <vector>
#include <QObject>
#include <utility>

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

    static std::unique_ptr<CadItem> createCadItemForEntity(DRW_Entity* entity);

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

    // 向文档中追加一个实体及其对应图元
    CadItem* appendEntity(std::unique_ptr<DRW_Entity> entity, std::unique_ptr<CadItem> item = nullptr);

    // 批量追加实体，可选择直接替换当前文档内容
    int appendEntities(std::vector<std::unique_ptr<DRW_Entity>> entities, bool replaceExisting);

    // 从文档中取出一个图元，并将所有权交还给调用方
    std::pair<std::unique_ptr<DRW_Entity>, std::unique_ptr<CadItem>> takeEntity(CadItem* item);

    // 当底层实体数据变更后，重建图元几何并通知视图刷新
    bool refreshEntity(CadItem* item);

    // 查询图元是否仍属于当前文档
    bool containsEntity(const CadItem* item) const;

signals:
    void sceneChanged();

public:
    // 原始文档数据
    std::unique_ptr<dx_data> m_data;
    // 内置实体数据数组
    std::vector<std::unique_ptr<CadItem>> m_entities;
};
