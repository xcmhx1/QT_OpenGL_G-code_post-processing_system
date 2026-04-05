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

std::unique_ptr<CadItem> createCadItem(DRW_Entity* entity)
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

        if (std::unique_ptr<CadItem> item = createCadItem(entity))
        {
            m_entities.push_back(std::move(item));
        }
    }
}


