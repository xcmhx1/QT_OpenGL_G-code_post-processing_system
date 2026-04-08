// 实现 CadDocument 模块，对应头文件中声明的主要行为和协作流程。
// 文档模型模块，负责持有原始 DXF 数据、内部图元容器以及场景变更通知。
#include "pch.h"

#include "CadDocument.h"

#include "CadArcItem.h"
#include "CadCircleItem.h"
#include "CadEllipseItem.h"
#include "CadLineItem.h"
#include "CadLWPolylineItem.h"
#include "CadPointItem.h"
#include "CadPolylineItem.h"
#include "dx_data.h"
#include "dx_iface.h"

#include <QDebug>
#include <QSet>

namespace
{
    QColor colorFromAci(int index)
    {
        static const QRgb aciStandardColors[] =
        {
            qRgb(0, 0, 0),
            qRgb(255, 0, 0),
            qRgb(255, 255, 0),
            qRgb(0, 255, 0),
            qRgb(0, 255, 255),
            qRgb(0, 0, 255),
            qRgb(255, 0, 255),
            qRgb(255, 255, 255),
            qRgb(128, 128, 128),
            qRgb(192, 192, 192)
        };

        if (index >= 1 && index <= 9)
        {
            return QColor(aciStandardColors[index]);
        }

        if (index == 0)
        {
            return QColor(Qt::white);
        }

        return QColor();
    }

    QColor colorFromTrueColor(int color24)
    {
        if (color24 < 0)
        {
            return QColor();
        }

        return QColor((color24 >> 16) & 0xFF, (color24 >> 8) & 0xFF, color24 & 0xFF);
    }

    QColor resolveEntityDisplayColor(const CadDocument& document, const CadItem* item)
    {
        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            return QColor(Qt::white);
        }

        const QColor trueColor = colorFromTrueColor(item->m_nativeEntity->color24);

        if (trueColor.isValid())
        {
            return trueColor;
        }

        if (item->m_nativeEntity->color == DRW::ColorByLayer)
        {
            return document.layerColor(QString::fromUtf8(item->m_nativeEntity->layer.c_str()), item->m_color);
        }

        const QColor indexColor = colorFromAci(item->m_nativeEntity->color);
        return indexColor.isValid() ? indexColor : item->m_color;
    }
}

std::unique_ptr<CadItem> CadDocument::createCadItemForEntity(DRW_Entity* entity)
{
    // 文档层把原始 DXF 实体适配为项目内部图元对象。
    // 这里是“解析数据 -> 可渲染/可编辑对象”的唯一分发入口。
    if (!entity)
    {
        return nullptr;
    }

    switch (entity->eType)
    {
    // 每个受支持的实体类型都映射到一个对应的 Cad*Item 派生类。
    case DRW::ETYPE::LINE:
        return std::make_unique<CadLineItem>(entity);

    case DRW::ETYPE::CIRCLE:
        return std::make_unique<CadCircleItem>(entity);

    case DRW::ETYPE::ARC:
        return std::make_unique<CadArcItem>(entity);

    case DRW::ETYPE::ELLIPSE:
        return std::make_unique<CadEllipseItem>(entity);

    case DRW::ETYPE::LWPOLYLINE:
        return std::make_unique<CadLWPolylineItem>(entity);

    case DRW::ETYPE::POINT:
        return std::make_unique<CadPointItem>(entity);

    case DRW::ETYPE::POLYLINE:
        return std::make_unique<CadPolylineItem>(entity);

    default:
        // 未适配的实体类型暂时不进入当前场景图元系统。
        return nullptr;
    }
}


CadDocument::CadDocument(QObject* parent)
    : QObject(parent)
{
    clearAll();
}

CadDocument::~CadDocument()
{
    clearAll();
}

void CadDocument::readDxfDocument(const QString& filePath)
{
    // 导入新文件前先清空现有文档，避免旧实体与新实体混杂。
    clearAll();

    // dx_iface 负责把文件内容解析进 dx_data。
    std::make_unique<dx_iface>()->fileImport(filePath.toLocal8Bit().constData(), m_data.get(), false);

    // 解析完成后再把支持的原始实体转换为内部 CadItem。
    init();
    emit sceneChanged();

    qDebug() << "CadDocument::readDxfDocument() ->" << filePath << "导入成功";
}

void CadDocument::saveDxfDocument(const QString& filePath)
{
    Q_UNUSED(filePath);
}

void CadDocument::eportDxfDocument(const QString& filePath)
{
    Q_UNUSED(filePath);
}

void CadDocument::clearAll()
{
    // m_entities 清空后会释放所有内部图元；
    // 重新创建 dx_data 则会重置原始解析结果容器。
    m_entities.clear();
    m_data = std::make_unique<dx_data>();
    ensureLayerExists(QStringLiteral("0"));
}

void CadDocument::init()
{
    // 当前只从模型空间实体列表构建内部图元。
    for (auto* entity : m_data->mBlock->ent)
    {
        if (!entity)
        {
            continue;
        }

        if (std::unique_ptr<CadItem> item = createCadItemForEntity(entity))
        {
            // 只有成功适配的实体才会进入场景图元数组。
            item->m_color = resolveEntityDisplayColor(*this, item.get());
            m_entities.push_back(std::move(item));
        }
    }
}

CadItem* CadDocument::appendEntity(std::unique_ptr<DRW_Entity> entity, std::unique_ptr<CadItem> item)
{
    if (entity == nullptr)
    {
        return nullptr;
    }

    if (item == nullptr)
    {
        // 调用方只给了原始实体时，这里自动补建对应 CadItem。
        item = createCadItemForEntity(entity.get());
    }

    if (item == nullptr)
    {
        return nullptr;
    }

    // m_data 持有原始实体，m_entities 持有内部图元，两者通过指针关联。
    DRW_Entity* nativeEntity = entity.release();
    CadItem* rawItem = item.get();

    ensureLayerExists(QString::fromUtf8(nativeEntity->layer.c_str()));
    rawItem->m_color = resolveEntityDisplayColor(*this, rawItem);
    m_data->mBlock->ent.push_back(nativeEntity);
    m_entities.push_back(std::move(item));

    emit sceneChanged();
    return rawItem;
}

int CadDocument::appendEntities(std::vector<std::unique_ptr<DRW_Entity>> entities, bool replaceExisting)
{
    int appendedCount = 0;

    if (replaceExisting)
    {
        clearAll();
    }

    for (std::unique_ptr<DRW_Entity>& entity : entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        std::unique_ptr<CadItem> item = createCadItemForEntity(entity.get());

        if (item == nullptr)
        {
            continue;
        }

        ensureLayerExists(QString::fromUtf8(entity->layer.c_str()));
        item->m_color = resolveEntityDisplayColor(*this, item.get());
        m_data->mBlock->ent.push_back(entity.release());
        m_entities.push_back(std::move(item));
        ++appendedCount;
    }

    if (replaceExisting || appendedCount > 0)
    {
        emit sceneChanged();
    }

    return appendedCount;
}

std::pair<std::unique_ptr<DRW_Entity>, std::unique_ptr<CadItem>> CadDocument::takeEntity(CadItem* item)
{
    // 删除/撤销等操作需要同时取回“原始实体 + 内部图元”这对对象。
    if (item == nullptr)
    {
        return {};
    }

    // 先在内部图元数组中定位对象，再到原始实体列表中定位其 nativeEntity。
    const auto itemIt = std::find_if
    (
        m_entities.begin(),
        m_entities.end(),
        [item](const std::unique_ptr<CadItem>& candidate)
        {
            return candidate.get() == item;
        }
    );

    if (itemIt == m_entities.end())
    {
        return {};
    }

    const auto nativeIt = std::find(m_data->mBlock->ent.begin(), m_data->mBlock->ent.end(), item->m_nativeEntity);

    if (nativeIt == m_data->mBlock->ent.end())
    {
        return {};
    }

    // 这里把原始指针重新包装回 unique_ptr，便于后续命令对象接管所有权。
    std::unique_ptr<DRW_Entity> entity(*nativeIt);
    std::unique_ptr<CadItem> removedItem = std::move(*itemIt);

    m_data->mBlock->ent.erase(nativeIt);
    m_entities.erase(itemIt);

    emit sceneChanged();
    return { std::move(entity), std::move(removedItem) };
}

bool CadDocument::refreshEntity(CadItem* item)
{
    // 原始实体被改动后，需要重新生成离散几何、方向和最终显示颜色。
    if (!containsEntity(item))
    {
        return false;
    }

    item->buildGeometryDatay();
    item->buildProcessDirection();
    item->m_color = resolveEntityDisplayColor(*this, item);

    emit sceneChanged();
    return true;
}

void CadDocument::notifySceneChanged()
{
    emit sceneChanged();
}

bool CadDocument::containsEntity(const CadItem* item) const
{
    // 通过地址判断图元是否仍属于当前文档，供编辑命令做安全检查。
    return std::any_of
    (
        m_entities.begin(),
        m_entities.end(),
        [item](const std::unique_ptr<CadItem>& candidate)
        {
            return candidate.get() == item;
        }
    );
}

QStringList CadDocument::layerNames() const
{
    QStringList layers;
    QSet<QString> seenLayers;

    const auto appendLayerName =
        [&layers, &seenLayers](const QString& layerName)
        {
            const QString normalizedLayerName = layerName.trimmed().isEmpty()
                ? QStringLiteral("0")
                : layerName.trimmed();

            if (seenLayers.contains(normalizedLayerName))
            {
                return;
            }

            seenLayers.insert(normalizedLayerName);
            layers.push_back(normalizedLayerName);
        };

    appendLayerName(QStringLiteral("0"));

    if (m_data != nullptr)
    {
        for (const DRW_Layer& layer : m_data->layers)
        {
            appendLayerName(QString::fromUtf8(layer.name.c_str()));
        }
    }

    for (const std::unique_ptr<CadItem>& entity : m_entities)
    {
        if (entity == nullptr || entity->m_nativeEntity == nullptr)
        {
            continue;
        }

        appendLayerName(QString::fromUtf8(entity->m_nativeEntity->layer.c_str()));
    }

    return layers;
}

bool CadDocument::ensureLayerExists(const QString& layerName)
{
    if (m_data == nullptr)
    {
        return false;
    }

    const QString normalizedLayerName = layerName.trimmed().isEmpty()
        ? QStringLiteral("0")
        : layerName.trimmed();

    for (const DRW_Layer& layer : m_data->layers)
    {
        if (QString::fromUtf8(layer.name.c_str()).compare(normalizedLayerName, Qt::CaseSensitive) == 0)
        {
            return false;
        }
    }

    DRW_Layer layer;
    layer.name = normalizedLayerName.toUtf8().constData();
    m_data->layers.push_back(layer);
    return true;
}

QColor CadDocument::layerColor(const QString& layerName, const QColor& fallback) const
{
    if (m_data == nullptr)
    {
        return fallback;
    }

    const QString normalizedLayerName = layerName.trimmed().isEmpty()
        ? QStringLiteral("0")
        : layerName.trimmed();

    for (const DRW_Layer& layer : m_data->layers)
    {
        if (QString::fromUtf8(layer.name.c_str()).compare(normalizedLayerName, Qt::CaseSensitive) != 0)
        {
            continue;
        }

        const QColor trueColor = colorFromTrueColor(layer.color24);

        if (trueColor.isValid())
        {
            return trueColor;
        }

        const QColor indexColor = colorFromAci(layer.color);
        return indexColor.isValid() ? indexColor : fallback;
    }

    return fallback;
}


