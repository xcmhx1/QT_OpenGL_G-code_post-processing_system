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

std::unique_ptr<CadItem> CadDocument::createCadItemForEntity(DRW_Entity* entity)
{
    if (!entity)
    {
        return nullptr;
    }

    switch (entity->eType)
    {
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
    clearAll();

    std::make_unique<dx_iface>()->fileImport(filePath.toLocal8Bit().constData(), m_data.get(), false);

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
    m_entities.clear();
    m_data = std::make_unique<dx_data>();
}

void CadDocument::init()
{
    for (auto* entity : m_data->mBlock->ent)
    {
        if (!entity)
        {
            continue;
        }

        if (std::unique_ptr<CadItem> item = createCadItemForEntity(entity))
        {
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
        item = createCadItemForEntity(entity.get());
    }

    if (item == nullptr)
    {
        return nullptr;
    }

    DRW_Entity* nativeEntity = entity.release();
    CadItem* rawItem = item.get();

    m_data->mBlock->ent.push_back(nativeEntity);
    m_entities.push_back(std::move(item));

    emit sceneChanged();
    return rawItem;
}

std::pair<std::unique_ptr<DRW_Entity>, std::unique_ptr<CadItem>> CadDocument::takeEntity(CadItem* item)
{
    if (item == nullptr)
    {
        return {};
    }

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

    std::unique_ptr<DRW_Entity> entity(*nativeIt);
    std::unique_ptr<CadItem> removedItem = std::move(*itemIt);

    m_data->mBlock->ent.erase(nativeIt);
    m_entities.erase(itemIt);

    emit sceneChanged();
    return { std::move(entity), std::move(removedItem) };
}

bool CadDocument::refreshEntity(CadItem* item)
{
    if (!containsEntity(item))
    {
        return false;
    }

    item->buildGeometryDatay();
    item->buildProcessDirection();
    item->m_color = item->buildColor();

    emit sceneChanged();
    return true;
}

bool CadDocument::containsEntity(const CadItem* item) const
{
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


