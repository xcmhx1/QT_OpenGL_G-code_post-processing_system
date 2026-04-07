#include "pch.h"

// 实现 CadItem 模块，对应头文件中声明的主要行为和协作流程。
// 图元基类模块，定义原生实体绑定、几何缓存和公共图元行为。
#include "CadItem.h"

CadItem::CadItem(DRW_Entity* entity, QObject* parent)
    : QObject(parent)
    , m_nativeEntity(entity)
{
    // 没有原生实体时不继续初始化，派生类会在空状态下自行兜底。
    if (m_nativeEntity == nullptr)
    {
        return;
    }

    // 基类构造时先缓存类型和颜色，几何则由派生类负责生成。
    m_type = m_nativeEntity->eType;
    m_color = buildColor();
}

void CadItem::buildProcessDirection()
{
    // 默认先清空方向，避免旧缓存污染。
    m_processDirection = QVector3D();

    // 少于两个顶点时无法定义方向，例如点图元就是这种情况。
    if (m_geometry.vertices.size() < 2)
    {
        return;
    }

    // 以首个顶点为基准，寻找第一条有效边作为加工方向。
    const QVector3D& start = m_geometry.vertices.front();

    for (int i = 1; i < m_geometry.vertices.size(); ++i)
    {
        QVector3D direction = m_geometry.vertices.at(i) - start;

        // 跳过零长度边，避免归一化无效向量。
        if (!qFuzzyIsNull(direction.lengthSquared()))
        {
            direction.normalize();
            // 反向加工时直接翻转单位方向。
            m_processDirection = m_isReverse ? -direction : direction;
            return;
        }
    }
}

QColor CadItem::buildColor()
{
    // 没有实体数据时统一退回白色，保证渲染层可用。
    if (m_nativeEntity == nullptr)
    {
        return QColor(Qt::white);
    }

    // true color 优先级最高，存在时直接使用。
    if (m_nativeEntity->color24 != -1)
    {
        return colorFromTrueColor();
    }

    // AutoCAD 中 256 表示 ByLayer，这里优先尝试解析图层颜色。
    if (m_nativeEntity->color == 256)
    {
        const QColor layerColor = colorFromLayer();
        return layerColor.isValid() ? layerColor : QColor(Qt::white);
    }

    // 其余情况按 ACI 索引色解析，并保留白色兜底。
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

    // 0 常被视为未指定/默认色，这里回落为白色以保证可见性。
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

    // 当前只显式维护常见基础色，超出范围时统一退回白色。
    return QColor(Qt::white);
}

QColor CadItem::colorFromTrueColor()
{
    if (m_nativeEntity == nullptr)
    {
        return QColor();
    }

    const int c24 = m_nativeEntity->color24;

    // -1 表示当前实体没有显式 true color。
    if (c24 == -1)
    {
        return QColor();
    }

    // color24 按 0xRRGGBB 存储，这里拆成 Qt 的 RGB 颜色对象。
    const int r = (c24 >> 16) & 0xFF;
    const int g = (c24 >> 8) & 0xFF;
    const int b = c24 & 0xFF;

    return QColor(r, g, b);
}

QColor CadItem::colorFromLayer()
{
    // 当前 CadItem 不持有图层表，无法在此解析图层颜色。
    // 后续若文档层向图元注入图层信息，可在这里改成真实解析逻辑。
    return QColor(Qt::white);
}

