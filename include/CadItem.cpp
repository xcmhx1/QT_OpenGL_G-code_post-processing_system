#include "pch.h"

#include "CadItem.h"

CadItem::CadItem(DRW_Entity* entity, QObject* parent)
    : QObject(parent)
    , m_nativeEntity(entity)
{
    m_type = m_nativeEntity->eType;
    m_color = buildColor();
}

void CadItem::buildProcessDirection()
{
}

QColor CadItem::buildColor()
{
    if (m_nativeEntity == nullptr)
    {
        return QColor(Qt::white);
    }

    if (m_nativeEntity->color24 != -1)
    {
        return colorFromTrueColor();
    }

    if (m_nativeEntity->color == 256)
    {
        const QColor layerColor = colorFromLayer();
        return layerColor.isValid() ? layerColor : QColor(Qt::white);
    }

    const QColor indexColor = colorFromIndex();
    return indexColor.isValid() ? indexColor : QColor(Qt::white);
}

QColor CadItem::colorFromIndex()
{
    if (m_nativeEntity == nullptr)
    {
        return QColor();
    }

    const int index = m_nativeEntity->color;

    if (index == 0)
    {
        return Qt::white;
    }

    static const QRgb aciStandardColors[]
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

    return QColor(Qt::white);
}

QColor CadItem::colorFromTrueColor()
{
    if (m_nativeEntity == nullptr)
    {
        return QColor();
    }

    const int c24 = m_nativeEntity->color24;

    if (c24 == -1)
    {
        return QColor();
    }

    const int r = (c24 >> 16) & 0xFF;
    const int g = (c24 >> 8) & 0xFF;
    const int b = c24 & 0xFF;

    return QColor(r, g, b);
}

QColor CadItem::colorFromLayer()
{
    // 当前 CadItem 不持有图层表，无法在此解析图层颜色。
    return QColor(Qt::white);
}



