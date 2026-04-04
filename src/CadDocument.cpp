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
    qDeleteAll(m_entities);
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

        if (CadItem* item = createCadItem(entity))
        {
            m_entities.append(item);
        }
    }
}

CadItem* CadDocument::createCadItem(DRW_Entity* entity)
{
    switch (entity->eType)
    {
    case DRW::ETYPE::LINE:
        return new CadLineItem(entity);
    case DRW::ETYPE::CIRCLE:
        return new CadCircleItem(entity);
    case DRW::ETYPE::ARC:
        return new CadArcItem(entity);
    case DRW::ETYPE::ELLIPSE:
        return new CadEllipseItem(entity);
    case DRW::ETYPE::LWPOLYLINE:
        return new CadLWPolylineItem(entity);
    case DRW::ETYPE::POINT:
        return new CadPointItem(entity);
    case DRW::ETYPE::POLYLINE:
        return new CadPolylineItem(entity);
    default:
        return nullptr;
    }
}
