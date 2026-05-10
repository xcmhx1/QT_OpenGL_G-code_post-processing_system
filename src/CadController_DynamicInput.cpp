// CadController 动态输入与命令提示实现
#include "pch.h"

#include "CadController.h"

#include "CadEditer.h"
#include "CadItem.h"
#include "CadViewer.h"

#include <algorithm>
#include <cmath>

namespace
{
    QVector3D flattenToDrawingPlane(const QVector3D& point)
    {
        return QVector3D(point.x(), point.y(), 0.0f);
    }

    QString drawTypeName(DrawType drawType)
    {
        switch (drawType)
        {
        case DrawType::Point:
            return QStringLiteral("点");
        case DrawType::Line:
            return QStringLiteral("直线");
        case DrawType::Xline:
            return QStringLiteral("构造线");
        case DrawType::Rectangle:
            return QStringLiteral("矩形");
        case DrawType::Polygon:
            return QStringLiteral("多边形");
        case DrawType::Circle:
            return QStringLiteral("圆");
        case DrawType::Arc:
            return QStringLiteral("圆弧");
        case DrawType::Ellipse:
            return QStringLiteral("椭圆");
        case DrawType::Polyline:
            return QStringLiteral("多段线");
        case DrawType::LWPolyline:
            return QStringLiteral("轻量多段线");
        default:
            return QStringLiteral("空闲");
        }
    }

    QString polygonConstructionModeText(bool circumscribedAboutCircle)
    {
        return circumscribedAboutCircle
            ? QStringLiteral("外切于圆")
            : QStringLiteral("内切于圆");
    }

    bool tryParseCoordinatePair(const QString& text, double& first, double& second)
    {
        const QStringList parts = text.split(QLatin1Char(','), Qt::KeepEmptyParts);

        if (parts.size() != 2)
        {
            return false;
        }

        bool firstOk = false;
        bool secondOk = false;
        const double firstValue = parts.at(0).toDouble(&firstOk);
        const double secondValue = parts.at(1).toDouble(&secondOk);

        if (!firstOk || !secondOk)
        {
            return false;
        }

        first = firstValue;
        second = secondValue;
        return true;
    }

    bool appendInfiniteEntityReferenceDistance(const CadItem* item, const QVector3D& basePoint, double& maxDistance)
    {
        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            return false;
        }

        if (item->m_type != DRW::ETYPE::RAY && item->m_type != DRW::ETYPE::XLINE)
        {
            return false;
        }

        const DRW_Ray* ray = static_cast<const DRW_Ray*>(item->m_nativeEntity);
        const QVector3D rayBase(
            static_cast<float>(ray->basePoint.x),
            static_cast<float>(ray->basePoint.y),
            static_cast<float>(ray->basePoint.z));
        maxDistance = std::max(maxDistance, static_cast<double>((flattenToDrawingPlane(rayBase) - basePoint).length()));
        return true;
    }

    double rotationAngleFromBaseToPoint(const QVector3D& basePoint, const QVector3D& worldPoint)
    {
        constexpr double kDegreesPerRadian = 180.0 / 3.14159265358979323846;
        const QVector3D delta = flattenToDrawingPlane(worldPoint) - flattenToDrawingPlane(basePoint);

        if (delta.lengthSquared() <= 0.000001f)
        {
            return 0.0;
        }

        return std::atan2(static_cast<double>(delta.y()), static_cast<double>(delta.x())) * kDegreesPerRadian;
    }

    double sideSignForLineLikeEntity(const QVector3D& firstPoint, const QVector3D& secondPoint, const QVector3D& sidePoint)
    {
        const QVector3D direction = flattenToDrawingPlane(secondPoint) - flattenToDrawingPlane(firstPoint);
        const QVector3D offset = flattenToDrawingPlane(sidePoint) - flattenToDrawingPlane(firstPoint);
        const double cross = static_cast<double>(direction.x()) * static_cast<double>(offset.y())
            - static_cast<double>(direction.y()) * static_cast<double>(offset.x());
        return cross >= 0.0 ? 1.0 : -1.0;
    }

    double signedOffsetDistanceForSide(const CadItem* item, double distance, const QVector3D& sidePoint)
    {
        const double absoluteDistance = std::abs(distance);

        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            return absoluteDistance;
        }

        if (item->m_type == DRW::ETYPE::LINE)
        {
            const DRW_Line* line = static_cast<const DRW_Line*>(item->m_nativeEntity);
            return absoluteDistance * sideSignForLineLikeEntity
            (
                QVector3D(line->basePoint.x, line->basePoint.y, line->basePoint.z),
                QVector3D(line->secPoint.x, line->secPoint.y, line->secPoint.z),
                sidePoint
            );
        }

        if (item->m_type == DRW::ETYPE::XLINE || item->m_type == DRW::ETYPE::RAY)
        {
            const DRW_Ray* ray = static_cast<const DRW_Ray*>(item->m_nativeEntity);
            const QVector3D basePoint(ray->basePoint.x, ray->basePoint.y, ray->basePoint.z);
            const QVector3D direction(ray->secPoint.x, ray->secPoint.y, ray->secPoint.z);
            return absoluteDistance * sideSignForLineLikeEntity(basePoint, basePoint + direction, sidePoint);
        }

        if (item->m_type == DRW::ETYPE::CIRCLE || item->m_type == DRW::ETYPE::ARC)
        {
            const DRW_Circle* circle = static_cast<const DRW_Circle*>(item->m_nativeEntity);
            const QVector3D center(circle->basePoint.x, circle->basePoint.y, circle->basePoint.z);
            const double sideRadius = static_cast<double>((flattenToDrawingPlane(sidePoint) - flattenToDrawingPlane(center)).length());
            return sideRadius >= circle->radious ? absoluteDistance : -absoluteDistance;
        }

        if (item->m_geometry.vertices.size() >= 2)
        {
            return absoluteDistance * sideSignForLineLikeEntity(item->m_geometry.vertices.front(), item->m_geometry.vertices.back(), sidePoint);
        }

        return absoluteDistance;
    }

    double scaleReferenceDistance(const QVector<CadItem*>& items, const QVector3D& basePoint)
    {
        double maxDistance = 0.0;

        for (const CadItem* item : items)
        {
            if (item == nullptr)
            {
                continue;
            }

            if (appendInfiniteEntityReferenceDistance(item, basePoint, maxDistance))
            {
                continue;
            }

            for (const QVector3D& vertex : item->m_geometry.vertices)
            {
                maxDistance = std::max(maxDistance, static_cast<double>((flattenToDrawingPlane(vertex) - basePoint).length()));
            }
        }

        return std::max(maxDistance, 1.0);
    }

    struct DynamicCommandDefinition
    {
        QString canonical;
        QString displayName;
        QStringList aliases;
    };

    QString normalizedCommandToken(const QString& token)
    {
        return token.trimmed().toLower();
    }

    const QVector<DynamicCommandDefinition>& dynamicCommandDefinitions()
    {
        static const QVector<DynamicCommandDefinition> definitions
        {
            { QStringLiteral("line"),       QStringLiteral("LINE  直线"),        { QStringLiteral("l"), QStringLiteral("line"), QStringLiteral("直线") } },
            { QStringLiteral("xline"),      QStringLiteral("XLINE 构造线"),      { QStringLiteral("x"), QStringLiteral("xl"), QStringLiteral("xline"), QStringLiteral("constructionline"), QStringLiteral("构造线"), QStringLiteral("无限线") } },
            { QStringLiteral("rectangle"),  QStringLiteral("RECTANGLE 矩形"),    { QStringLiteral("r"), QStringLiteral("rect"), QStringLiteral("rectangle"), QStringLiteral("矩形") } },
            { QStringLiteral("polygon"),    QStringLiteral("POLYGON 多边形"),    { QStringLiteral("g"), QStringLiteral("pg"), QStringLiteral("polygon"), QStringLiteral("多边形"), QStringLiteral("正多边形") } },
            { QStringLiteral("point"),      QStringLiteral("POINT 点"),          { QStringLiteral("p"), QStringLiteral("point"), QStringLiteral("点") } },
            { QStringLiteral("circle"),     QStringLiteral("CIRCLE 圆"),         { QStringLiteral("c"), QStringLiteral("circle"), QStringLiteral("圆") } },
            { QStringLiteral("arc"),        QStringLiteral("ARC   圆弧"),        { QStringLiteral("a"), QStringLiteral("arc"), QStringLiteral("圆弧") } },
            { QStringLiteral("ellipse"),    QStringLiteral("ELLIPSE 椭圆"),      { QStringLiteral("e"), QStringLiteral("ellipse"), QStringLiteral("椭圆") } },
            { QStringLiteral("polyline"),   QStringLiteral("POLYLINE 多段线"),   { QStringLiteral("o"), QStringLiteral("polyline"), QStringLiteral("pline"), QStringLiteral("多段线") } },
            { QStringLiteral("lwpolyline"), QStringLiteral("LWPOLYLINE 轻量多段线"), { QStringLiteral("w"), QStringLiteral("lwpolyline"), QStringLiteral("轻量多段线") } },
            { QStringLiteral("move"),       QStringLiteral("MOVE  移动"),        { QStringLiteral("m"), QStringLiteral("move"), QStringLiteral("移动") } },
            { QStringLiteral("copy"),       QStringLiteral("COPY  复制"),        { QStringLiteral("co"), QStringLiteral("cp"), QStringLiteral("copy"), QStringLiteral("复制") } },
            { QStringLiteral("rotate"),     QStringLiteral("ROTATE 旋转"),        { QStringLiteral("ro"), QStringLiteral("rotate"), QStringLiteral("旋转") } },
            { QStringLiteral("scale"),      QStringLiteral("SCALE 缩放"),        { QStringLiteral("sc"), QStringLiteral("scale"), QStringLiteral("缩放") } },
            { QStringLiteral("mirror"),     QStringLiteral("MIRROR 镜像"),        { QStringLiteral("mi"), QStringLiteral("mirror"), QStringLiteral("镜像") } },
            { QStringLiteral("offset"),     QStringLiteral("OFFSET 偏移"),        { QStringLiteral("o"), QStringLiteral("offset"), QStringLiteral("偏移") } },
            { QStringLiteral("arrayrect"),  QStringLiteral("ARRAYRECT 矩形阵列"), { QStringLiteral("arrayrect"), QStringLiteral("ar"), QStringLiteral("矩形阵列") } },
            { QStringLiteral("arraypolar"), QStringLiteral("ARRAYPOLAR 环形阵列"), { QStringLiteral("arraypolar"), QStringLiteral("环形阵列") } },
            { QStringLiteral("trim"),       QStringLiteral("TRIM  修剪"),        { QStringLiteral("tr"), QStringLiteral("trim"), QStringLiteral("修剪") } },
            { QStringLiteral("extend"),     QStringLiteral("EXTEND 延申"),        { QStringLiteral("ex"), QStringLiteral("extend"), QStringLiteral("延申"), QStringLiteral("延伸") } },
            { QStringLiteral("join"),       QStringLiteral("JOIN  合并"),        { QStringLiteral("j"), QStringLiteral("join"), QStringLiteral("合并") } },
            { QStringLiteral("fillet"),     QStringLiteral("FILLET 圆角"),        { QStringLiteral("f"), QStringLiteral("fillet"), QStringLiteral("圆角") } },
            { QStringLiteral("chamfer"),    QStringLiteral("CHAMFER 倒角"),       { QStringLiteral("cha"), QStringLiteral("chamfer"), QStringLiteral("倒角"), QStringLiteral("直角") } },
            { QStringLiteral("delete"),     QStringLiteral("DELETE 删除"),       { QStringLiteral("del"), QStringLiteral("delete"), QStringLiteral("erase"), QStringLiteral("删除") } },
            { QStringLiteral("color"),      QStringLiteral("COLOR 改色"),        { QStringLiteral("k"), QStringLiteral("color"), QStringLiteral("改色"), QStringLiteral("颜色") } },
            { QStringLiteral("fit"),        QStringLiteral("FIT   适配视图"),    { QStringLiteral("f"), QStringLiteral("fit"), QStringLiteral("zoomextents"), QStringLiteral("适配") } },
            { QStringLiteral("top"),        QStringLiteral("TOP   顶视图"),      { QStringLiteral("t"), QStringLiteral("top"), QStringLiteral("home"), QStringLiteral("顶视图") } },
            { QStringLiteral("zoomin"),     QStringLiteral("ZOOMIN  放大"),      { QStringLiteral("zoomin"), QStringLiteral("zin"), QStringLiteral("放大") } },
            { QStringLiteral("zoomout"),    QStringLiteral("ZOOMOUT 缩小"),      { QStringLiteral("zoomout"), QStringLiteral("zout"), QStringLiteral("缩小") } },
        };

        return definitions;
    }

    bool commandAliasMatches(const DynamicCommandDefinition& definition, const QString& normalizedInput)
    {
        if (normalizedInput.isEmpty())
        {
            return true;
        }

        if (normalizedCommandToken(definition.canonical).startsWith(normalizedInput))
        {
            return true;
        }

        for (const QString& alias : definition.aliases)
        {
            if (normalizedCommandToken(alias).startsWith(normalizedInput))
            {
                return true;
            }
        }

        return false;
    }
}

bool CadController::isParameterInputCommandActive() const
{
    return m_drawState.editType == EditType::ParameterInput
        && m_parameterInputSession.command != ParameterInputCommand::None;
}

bool CadController::isAwaitingParameterFieldInput() const
{
    if (!isParameterInputCommandActive())
    {
        return false;
    }

    return !isAwaitingPointInput();
}

QString CadController::parameterInputTitle() const
{
    switch (m_parameterInputSession.command)
    {
    case ParameterInputCommand::Polygon:
        return QStringLiteral("多边形");
    case ParameterInputCommand::Copy:
        return QStringLiteral("复制");
    case ParameterInputCommand::Rotate:
        return QStringLiteral("旋转");
    case ParameterInputCommand::Scale:
        return QStringLiteral("缩放");
    case ParameterInputCommand::RectangularArray:
        return QStringLiteral("矩形阵列");
    case ParameterInputCommand::CircularArray:
        return QStringLiteral("环形阵列");
    case ParameterInputCommand::Mirror:
        return QStringLiteral("镜像");
    case ParameterInputCommand::Offset:
        return QStringLiteral("偏移");
    case ParameterInputCommand::Trim:
        return QStringLiteral("修剪");
    case ParameterInputCommand::Extend:
        return QStringLiteral("延申");
    case ParameterInputCommand::Fillet:
        return QStringLiteral("圆角");
    case ParameterInputCommand::Chamfer:
        return QStringLiteral("倒角");
    default:
        break;
    }

    return QStringLiteral("参数输入");
}

QString CadController::parameterInputPrompt() const
{
    switch (m_parameterInputSession.command)
    {
    case ParameterInputCommand::Polygon:
        return m_parameterInputSession.stageIndex == 0
            ? QStringLiteral("POLYGON: 输入边数（3-1024）")
            : QStringLiteral("POLYGON: 选择内切于圆或外切于圆");
    case ParameterInputCommand::Copy:
        return m_parameterInputSession.stageIndex == 0
            ? QStringLiteral("COPY: 指定基点")
            : QStringLiteral("COPY: 指定第二点或输入位移");
    case ParameterInputCommand::Rotate:
        return m_parameterInputSession.stageIndex == 0
            ? QStringLiteral("ROTATE: 指定旋转基点")
            : QStringLiteral("ROTATE: 指定旋转角度或输入角度");
    case ParameterInputCommand::Scale:
        return m_parameterInputSession.stageIndex == 0
            ? QStringLiteral("SCALE: 指定缩放基点")
            : QStringLiteral("SCALE: 指定缩放因子或输入倍率");
    case ParameterInputCommand::RectangularArray:
        switch (m_parameterInputSession.stageIndex)
        {
        case 0:
            return QStringLiteral("ARRAYRECT: 输入行数");
        case 1:
            return QStringLiteral("ARRAYRECT: 输入列数");
        case 2:
            return QStringLiteral("ARRAYRECT: 输入行间距（Y）");
        default:
            return QStringLiteral("ARRAYRECT: 输入列间距（X）");
        }
    case ParameterInputCommand::CircularArray:
        switch (m_parameterInputSession.stageIndex)
        {
        case 0:
            return QStringLiteral("ARRAYPOLAR: 输入项目总数");
        case 1:
            return QStringLiteral("ARRAYPOLAR: 输入填充角度");
        case 2:
            return QStringLiteral("ARRAYPOLAR: 指定阵列中心");
        default:
            return QStringLiteral("ARRAYPOLAR: 输入旋转方式");
        }
    case ParameterInputCommand::Mirror:
        switch (m_parameterInputSession.stageIndex)
        {
        case 0:
            return QStringLiteral("MIRROR: 指定镜像线第一点");
        case 1:
            return QStringLiteral("MIRROR: 指定镜像线第二点");
        default:
            return QStringLiteral("MIRROR: 输入是否删除原图元");
        }
    case ParameterInputCommand::Offset:
        return m_parameterInputSession.stageIndex == 0
            ? QStringLiteral("OFFSET: 输入偏移距离")
            : QStringLiteral("OFFSET: 指定偏移侧");
    case ParameterInputCommand::Trim:
        return QStringLiteral("TRIM: 输入要修剪的端点");
    case ParameterInputCommand::Extend:
        return QStringLiteral("EXTEND: 输入要延申的端点");
    case ParameterInputCommand::Fillet:
        return QStringLiteral("FILLET: 输入圆角半径");
    case ParameterInputCommand::Chamfer:
        return m_parameterInputSession.stageIndex == 0
            ? QStringLiteral("CHAMFER: 输入第一条边距离")
            : QStringLiteral("CHAMFER: 输入第二条边距离");
    default:
        break;
    }

    return QStringLiteral("参数输入");
}

QString CadController::currentPrompt() const
{
    QString basePrompt = QStringLiteral("无活动命令");

    if (isParameterInputCommandActive())
    {
        return appendDynamicInputPromptState(parameterInputPrompt());
    }

    if (m_drawState.editType == EditType::Move)
    {
        switch (m_drawState.moveSubMode)
        {
        case MoveEditSubMode::AwaitBasePoint:
            basePrompt = QStringLiteral("MOVE: 指定基点");
            break;
        case MoveEditSubMode::AwaitTargetPoint:
            basePrompt = QStringLiteral("MOVE: 指定目标点");
            break;
        default:
            basePrompt = QStringLiteral("MOVE");
            break;
        }

        return appendDynamicInputPromptState(basePrompt);
    }

    if (m_drawState.editType == EditType::GripEdit)
    {
        basePrompt = QStringLiteral("GRIP: 指定目标点");
        return appendDynamicInputPromptState(basePrompt);
    }

    if (!m_drawState.isDrawing)
    {
        return appendDynamicInputPromptState(basePrompt);
    }

    switch (m_drawState.drawType)
    {
    case DrawType::Point:
        basePrompt = QStringLiteral("POINT: 指定点位置");
        break;
    case DrawType::Line:
        basePrompt = m_drawState.lineSubMode == LineDrawSubMode::AwaitEndPoint
            ? QStringLiteral("LINE: 指定下一点")
            : QStringLiteral("LINE: 指定第一点");
        break;
    case DrawType::Xline:
        basePrompt = m_drawState.lineSubMode == LineDrawSubMode::AwaitEndPoint
            ? QStringLiteral("XLINE: 指定通过点")
            : QStringLiteral("XLINE: 指定基点");
        break;
    case DrawType::Rectangle:
        basePrompt = m_drawState.rectangleSubMode == RectangleDrawSubMode::AwaitSecondCorner
            ? QStringLiteral("RECTANGLE: 指定另一个角点")
            : QStringLiteral("RECTANGLE: 指定第一个角点");
        break;
    case DrawType::Polygon:
        basePrompt = QStringLiteral("POLYGON[%1边/%2]: %3")
            .arg(m_drawState.polygonSideCount)
            .arg(polygonConstructionModeText(m_drawState.polygonCircumscribedAboutCircle))
            .arg
            (
                m_drawState.polygonSubMode == PolygonDrawSubMode::AwaitRadius
                    ? QStringLiteral("指定参考圆半径")
                    : QStringLiteral("指定中心")
            );
        break;
    case DrawType::Circle:
        basePrompt = m_drawState.circleSubMode == CircleDrawSubMode::AwaitRadius
            ? QStringLiteral("CIRCLE: 指定半径")
            : QStringLiteral("CIRCLE: 指定圆心");
        break;
    case DrawType::Arc:
        switch (m_drawState.arcSubMode)
        {
        case ArcDrawSubMode::AwaitRadius:
            basePrompt = QStringLiteral("ARC: 指定起点");
            break;
        case ArcDrawSubMode::AwaitStartAngle:
            basePrompt = QStringLiteral("ARC: 指定起始角");
            break;
        case ArcDrawSubMode::AwaitEndAngle:
            basePrompt = QStringLiteral("ARC: 指定终点（按住 Ctrl 切换补弧）");
            break;
        default:
            basePrompt = QStringLiteral("ARC: 指定圆心");
            break;
        }
        break;
    case DrawType::Ellipse:
        switch (m_drawState.ellipseSubMode)
        {
        case EllipseDrawSubMode::AwaitMajorAxis:
            basePrompt = QStringLiteral("ELLIPSE: 指定长轴端点");
            break;
        case EllipseDrawSubMode::AwaitMinorAxis:
            basePrompt = QStringLiteral("ELLIPSE: 指定短轴距离");
            break;
        default:
            basePrompt = QStringLiteral("ELLIPSE: 指定中心");
            break;
        }
        break;
    case DrawType::Polyline:
        if (m_drawState.polylineSubMode == PolylineDrawSubMode::AwaitArcEndPoint)
        {
            basePrompt = QStringLiteral("POLYLINE[圆弧]: 指定圆弧终点，L切换直线，Enter/Space结束，C闭合");
            break;
        }

        if (m_drawState.polylineSubMode == PolylineDrawSubMode::AwaitLineEndPoint)
        {
            basePrompt = QStringLiteral("POLYLINE[直线]: 指定下一点，A切换圆弧，Enter/Space结束，C闭合");
            break;
        }

        basePrompt = QStringLiteral("POLYLINE: 指定第一点");
        break;
    case DrawType::LWPolyline:
        if (m_drawState.lwPolylineSubMode == LWPolylineDrawSubMode::AwaitArcEndPoint)
        {
            basePrompt = QStringLiteral("LWPOLYLINE[圆弧]: 指定圆弧终点，L切换直线，Enter/Space结束，C闭合");
            break;
        }

        if (m_drawState.lwPolylineSubMode == LWPolylineDrawSubMode::AwaitLineEndPoint)
        {
            basePrompt = QStringLiteral("LWPOLYLINE[直线]: 指定下一点，A切换圆弧，Enter/Space结束，C闭合");
            break;
        }

        basePrompt = QStringLiteral("LWPOLYLINE: 指定第一点");
        break;
    default:
        break;
    }

    return appendDynamicInputPromptState(basePrompt);
}

QString CadController::currentCommandName() const
{
    if (isParameterInputCommandActive())
    {
        return parameterInputTitle();
    }

    if (m_drawState.editType == EditType::Move)
    {
        return QStringLiteral("移动");
    }

    if (m_drawState.editType == EditType::GripEdit)
    {
        return QStringLiteral("控制点编辑");
    }

    return drawTypeName(m_drawState.drawType);
}

CadDynamicInputOverlayState CadController::dynamicInputOverlayState() const
{
    CadDynamicInputOverlayState state;

    if (!m_drawState.hasActiveCommand())
    {
        return state;
    }

    state.visible = true;
    state.title = currentCommandName();

    if (state.title.trimmed().isEmpty() || state.title == QStringLiteral("空闲"))
    {
        state.title = QStringLiteral("动态输入");
    }

    if (isParameterInputCommandActive() && isAwaitingParameterFieldInput())
    {
        QString labelText;
        QString valueText;
        QString stageHint = QStringLiteral("Enter/Space确认，Esc清空输入");

        switch (m_parameterInputSession.command)
        {
        case ParameterInputCommand::Polygon:
            if (m_parameterInputSession.stageIndex == 0)
            {
                labelText = QStringLiteral("边数");
                valueText = QString::number(m_parameterInputSession.intValue1);
                stageHint = QStringLiteral("输入 3-1024，Enter/Space确认，Esc清空输入");
            }
            else
            {
                labelText = QStringLiteral("构造方式");
                valueText = m_parameterInputSession.boolValue ? QStringLiteral("外切于圆") : QStringLiteral("内切于圆");
                stageHint = QStringLiteral("输入 内切/外切 或 0/1，Enter/Space确认");
            }
            break;
        case ParameterInputCommand::Copy:
            labelText = QStringLiteral("位移");
            valueText = QStringLiteral("%1, %2")
                .arg(formatDynamicInputValue(m_parameterInputSession.doubleValue1))
                .arg(formatDynamicInputValue(m_parameterInputSession.doubleValue2));
            stageHint = QStringLiteral("指定第二点，或输入相对坐标后 Enter/Space确认");
            break;
        case ParameterInputCommand::Rotate:
            labelText = QStringLiteral("旋转角度");
            valueText = formatDynamicInputValue(m_parameterInputSession.doubleValue1);
            stageHint = QStringLiteral("移动鼠标预览，左键或 Enter/Space确认，也可输入角度");
            break;
        case ParameterInputCommand::Scale:
            labelText = QStringLiteral("缩放倍率");
            valueText = formatDynamicInputValue(m_parameterInputSession.doubleValue1);
            stageHint = QStringLiteral("移动鼠标预览，左键或 Enter/Space确认，也可输入倍率");
            break;
        case ParameterInputCommand::RectangularArray:
            switch (m_parameterInputSession.stageIndex)
            {
            case 0:
                labelText = QStringLiteral("行数");
                valueText = QString::number(m_parameterInputSession.intValue1);
                stageHint = QStringLiteral("输入行数后 Enter/Space确认");
                break;
            case 1:
                labelText = QStringLiteral("列数");
                valueText = QString::number(m_parameterInputSession.intValue2);
                stageHint = QStringLiteral("输入列数后 Enter/Space确认");
                break;
            case 2:
                labelText = QStringLiteral("行间距");
                valueText = formatDynamicInputValue(m_parameterInputSession.doubleValue1);
                stageHint = QStringLiteral("输入 Y 方向间距后 Enter/Space确认");
                break;
            default:
                labelText = QStringLiteral("列间距");
                valueText = formatDynamicInputValue(m_parameterInputSession.doubleValue2);
                stageHint = QStringLiteral("输入 X 方向间距后 Enter/Space确认");
                break;
            }
            break;
        case ParameterInputCommand::CircularArray:
            switch (m_parameterInputSession.stageIndex)
            {
            case 0:
                labelText = QStringLiteral("项目总数");
                valueText = QString::number(m_parameterInputSession.intValue1);
                stageHint = QStringLiteral("输入项目总数后 Enter/Space确认");
                break;
            case 1:
                labelText = QStringLiteral("填充角度");
                valueText = formatDynamicInputValue(m_parameterInputSession.doubleValue1);
                stageHint = QStringLiteral("输入填充角度后 Enter/Space确认");
                break;
            default:
                labelText = QStringLiteral("旋转方式");
                valueText = m_parameterInputSession.boolValue ? QStringLiteral("旋转副本方向") : QStringLiteral("保持原方向");
                stageHint = QStringLiteral("输入 旋转/保持 或 0/1，Enter/Space确认");
                break;
            }
            break;
        case ParameterInputCommand::Mirror:
            labelText = QStringLiteral("删除原图元");
            valueText = m_parameterInputSession.boolValue ? QStringLiteral("是") : QStringLiteral("否");
            stageHint = QStringLiteral("输入 是/否 或 0/1，Enter/Space确认");
            break;
        case ParameterInputCommand::Offset:
            labelText = QStringLiteral("偏移距离");
            valueText = formatDynamicInputValue(m_parameterInputSession.doubleValue1);
            stageHint = m_parameterInputSession.stageIndex == 0
                ? QStringLiteral("输入距离后 Enter/Space确认")
                : QStringLiteral("点击图元偏移侧确认");
            break;
        case ParameterInputCommand::Trim:
        case ParameterInputCommand::Extend:
            labelText = QStringLiteral("目标端点");
            valueText = m_parameterInputSession.boolValue ? QStringLiteral("起点端") : QStringLiteral("终点端");
            stageHint = QStringLiteral("输入 起点/终点 或 0/1，Enter/Space确认");
            break;
        case ParameterInputCommand::Fillet:
            labelText = QStringLiteral("圆角半径");
            valueText = formatDynamicInputValue(m_parameterInputSession.doubleValue1);
            stageHint = QStringLiteral("输入半径后 Enter/Space确认");
            break;
        case ParameterInputCommand::Chamfer:
            labelText = m_parameterInputSession.stageIndex == 0 ? QStringLiteral("第一边距离") : QStringLiteral("第二边距离");
            valueText = formatDynamicInputValue(m_parameterInputSession.stageIndex == 0 ? m_parameterInputSession.doubleValue1 : m_parameterInputSession.doubleValue2);
            stageHint = QStringLiteral("输入距离后 Enter/Space确认");
            break;
        default:
            break;
        }

        if (!m_parameterInputSession.fieldBuffer.isEmpty())
        {
            valueText = m_parameterInputSession.fieldBuffer;
        }

        state.stageHint = stageHint;
        state.rows.push_back({ labelText, valueText, true });
        return state;
    }

    if (!isAwaitingPointInput())
    {
        return state;
    }

    state.stageHint = QStringLiteral("Tab切换字段，Enter/Space确认，Esc清空输入");
    state.expressionMode = m_drawState.dynamicInputExpressionMode;

    if (state.expressionMode)
    {
        state.rows.push_back({ QStringLiteral("表达式"), m_drawState.dynamicInputBuffer, true });
        state.expressionText = m_drawState.dynamicInputBuffer;
        return state;
    }

    const QVector3D previewPoint = applyPointDynamicFieldOverride(m_drawState.currentPos, true);
    QString xValueText = formatDynamicInputValue(previewPoint.x());
    QString yValueText = formatDynamicInputValue(previewPoint.y());
    const bool xActive = m_drawState.dynamicInputActiveFieldIndex == 0;
    const bool yActive = m_drawState.dynamicInputActiveFieldIndex == 1;

    if (!m_drawState.dynamicInputFieldBuffer.isEmpty())
    {
        if (xActive)
        {
            xValueText = m_drawState.dynamicInputFieldBuffer;
        }
        else
        {
            yValueText = m_drawState.dynamicInputFieldBuffer;
        }
    }

    state.rows.push_back
    (
        {
            m_drawState.dynamicInputXLocked ? QStringLiteral("X [锁]") : QStringLiteral("X"),
            xValueText,
            xActive
        }
    );
    state.rows.push_back
    (
        {
            m_drawState.dynamicInputYLocked ? QStringLiteral("Y [锁]") : QStringLiteral("Y"),
            yValueText,
            yActive
        }
    );

    return state;
}

CadDynamicCommandOverlayState CadController::dynamicCommandOverlayState() const
{
    CadDynamicCommandOverlayState state;

    if (!isDynamicCommandModeActive())
    {
        return state;
    }

    state.visible = true;
    state.inputText = m_drawState.dynamicCommandBuffer;
    state.hintText = QStringLiteral("Tab/Shift+Tab 选择，Enter/Space 执行，Esc 取消");

    const QVector<int> matchIndices = collectDynamicCommandMatchIndices();

    if (matchIndices.isEmpty())
    {
        state.candidates = { QStringLiteral("无匹配命令") };
        state.activeCandidateIndex = 0;
        return state;
    }

    const QVector<DynamicCommandDefinition>& definitions = dynamicCommandDefinitions();
    state.candidates.reserve(matchIndices.size());

    for (int matchIndex : matchIndices)
    {
        if (matchIndex >= 0 && matchIndex < definitions.size())
        {
            state.candidates.append(definitions.at(matchIndex).displayName);
        }
    }

    const int maxIndex = std::max(0, static_cast<int>(state.candidates.size()) - 1);
    state.activeCandidateIndex = std::clamp(m_drawState.dynamicCommandActiveIndex, 0, maxIndex);
    return state;
}

bool CadController::isDynamicCommandModeActive() const
{
    return !m_drawState.hasActiveCommand() && !m_drawState.dynamicCommandBuffer.trimmed().isEmpty();
}

void CadController::clearDynamicCommandMode()
{
    m_drawState.dynamicCommandBuffer.clear();
    m_drawState.dynamicCommandActiveIndex = 0;
}

QVector<int> CadController::collectDynamicCommandMatchIndices() const
{
    QVector<int> matches;

    if (m_drawState.hasActiveCommand())
    {
        return matches;
    }

    const QString normalizedInput = normalizedCommandToken(m_drawState.dynamicCommandBuffer);
    const QVector<DynamicCommandDefinition>& definitions = dynamicCommandDefinitions();

    for (int index = 0; index < definitions.size(); ++index)
    {
        if (commandAliasMatches(definitions.at(index), normalizedInput))
        {
            matches.append(index);
        }
    }

    return matches;
}

void CadController::normalizeDynamicCommandSelectionIndex()
{
    const QVector<int> matchIndices = collectDynamicCommandMatchIndices();

    if (matchIndices.isEmpty())
    {
        m_drawState.dynamicCommandActiveIndex = 0;
        return;
    }

    const int maxIndex = matchIndices.size() - 1;
    m_drawState.dynamicCommandActiveIndex = std::clamp(m_drawState.dynamicCommandActiveIndex, 0, maxIndex);
}

bool CadController::cycleDynamicCommandSelection(int step)
{
    if (!isDynamicCommandModeActive())
    {
        return false;
    }

    const QVector<int> matchIndices = collectDynamicCommandMatchIndices();

    if (matchIndices.isEmpty())
    {
        return true;
    }

    const int size = matchIndices.size();
    const int currentIndex = std::clamp(m_drawState.dynamicCommandActiveIndex, 0, size - 1);
    const int stepNormalized = step >= 0 ? 1 : -1;
    const int nextIndex = (currentIndex + stepNormalized + size) % size;
    m_drawState.dynamicCommandActiveIndex = nextIndex;

    if (m_viewer != nullptr)
    {
        m_viewer->refreshCommandPrompt();
        m_viewer->requestViewUpdate();
    }

    return true;
}

bool CadController::executeSelectedDynamicCommand()
{
    if (!isDynamicCommandModeActive())
    {
        return false;
    }

    const QVector<int> matchIndices = collectDynamicCommandMatchIndices();

    if (matchIndices.isEmpty())
    {
        if (m_viewer != nullptr)
        {
            m_viewer->appendCommandMessage(QStringLiteral("未找到可执行命令"));
            m_viewer->refreshCommandPrompt();
            m_viewer->requestViewUpdate();
        }

        return true;
    }

    const int selectedOrdinal = std::clamp(m_drawState.dynamicCommandActiveIndex, 0, static_cast<int>(matchIndices.size()) - 1);
    const int selectedIndex = matchIndices.at(selectedOrdinal);
    const QVector<DynamicCommandDefinition>& definitions = dynamicCommandDefinitions();
    const QString canonicalCommand = definitions.at(selectedIndex).canonical;
    clearDynamicCommandMode();
    const bool handled = executeIdleCommandByCanonical(canonicalCommand);

    if (!handled && m_viewer != nullptr)
    {
        m_viewer->appendCommandMessage(QStringLiteral("命令执行失败: %1").arg(canonicalCommand));
        m_viewer->refreshCommandPrompt();
        m_viewer->requestViewUpdate();
    }

    return true;
}

bool CadController::executeIdleCommandByCanonical(const QString& canonicalCommand)
{
    const QString normalized = normalizedCommandToken(canonicalCommand);

    if (normalized == QStringLiteral("point"))
    {
        beginDrawing(DrawType::Point, m_drawState.drawingColor);
        return true;
    }

    if (normalized == QStringLiteral("line"))
    {
        beginDrawing(DrawType::Line, m_drawState.drawingColor);
        return true;
    }

    if (normalized == QStringLiteral("xline"))
    {
        beginDrawing(DrawType::Xline, m_drawState.drawingColor);
        return true;
    }

    if (normalized == QStringLiteral("rectangle"))
    {
        beginDrawing(DrawType::Rectangle, m_drawState.drawingColor);
        return true;
    }

    if (normalized == QStringLiteral("polygon"))
    {
        beginDrawing(DrawType::Polygon, m_drawState.drawingColor);
        return true;
    }

    if (normalized == QStringLiteral("circle"))
    {
        beginDrawing(DrawType::Circle, m_drawState.drawingColor);
        return true;
    }

    if (normalized == QStringLiteral("arc"))
    {
        beginDrawing(DrawType::Arc, m_drawState.drawingColor);
        return true;
    }

    if (normalized == QStringLiteral("ellipse"))
    {
        beginDrawing(DrawType::Ellipse, m_drawState.drawingColor);
        return true;
    }

    if (normalized == QStringLiteral("polyline"))
    {
        beginDrawing(DrawType::Polyline, m_drawState.drawingColor);
        return true;
    }

    if (normalized == QStringLiteral("lwpolyline"))
    {
        beginDrawing(DrawType::LWPolyline, m_drawState.drawingColor);
        return true;
    }

    if (normalized == QStringLiteral("move"))
    {
        return beginMoveSelected();
    }

    if (normalized == QStringLiteral("copy"))
    {
        return beginCopySelected();
    }

    if (normalized == QStringLiteral("rotate"))
    {
        return beginRotateSelected();
    }

    if (normalized == QStringLiteral("scale"))
    {
        return beginScaleSelected();
    }

    if (normalized == QStringLiteral("mirror"))
    {
        return beginMirrorSelected();
    }

    if (normalized == QStringLiteral("offset"))
    {
        return beginOffsetSelected();
    }

    if (normalized == QStringLiteral("arrayrect"))
    {
        return beginRectangularArraySelected();
    }

    if (normalized == QStringLiteral("arraypolar"))
    {
        return beginCircularArraySelected();
    }

    if (normalized == QStringLiteral("trim"))
    {
        return beginTrimSelected();
    }

    if (normalized == QStringLiteral("extend"))
    {
        return beginExtendSelected();
    }

    if (normalized == QStringLiteral("join"))
    {
        return joinSelectedEntities();
    }

    if (normalized == QStringLiteral("fillet"))
    {
        return beginFilletSelected();
    }

    if (normalized == QStringLiteral("chamfer"))
    {
        return beginChamferSelected();
    }

    if (normalized == QStringLiteral("delete"))
    {
        return deleteSelectedEntity();
    }

    if (normalized == QStringLiteral("color"))
    {
        return changeSelectedEntityColor();
    }

    if (normalized == QStringLiteral("fit"))
    {
        if (m_viewer != nullptr)
        {
            m_viewer->fitSceneView();
            return true;
        }

        return false;
    }

    if (normalized == QStringLiteral("top"))
    {
        if (m_viewer != nullptr)
        {
            m_viewer->resetToTopView();
            return true;
        }

        return false;
    }

    if (normalized == QStringLiteral("zoomin"))
    {
        if (m_viewer != nullptr)
        {
            m_viewer->zoomIn();
            return true;
        }

        return false;
    }

    if (normalized == QStringLiteral("zoomout"))
    {
        if (m_viewer != nullptr)
        {
            m_viewer->zoomOut();
            return true;
        }

        return false;
    }

    return false;
}

void CadController::clearScalePreview()
{
    m_drawState.scalePreviewActive = false;
    m_drawState.scalePreviewBasePoint = QVector3D();
    m_drawState.scalePreviewFactor = 1.0;
}

void CadController::clearCopyPreview()
{
    m_drawState.copyPreviewActive = false;
    m_drawState.copyPreviewBasePoint = QVector3D();
    m_drawState.copyPreviewDelta = QVector3D();
}

void CadController::clearRotatePreview()
{
    m_drawState.rotatePreviewActive = false;
    m_drawState.rotatePreviewBasePoint = QVector3D();
    m_drawState.rotatePreviewAngleDegrees = 0.0;
}

void CadController::clearModifyPreviews()
{
    clearCopyPreview();
    clearRotatePreview();
    clearScalePreview();
    m_drawState.mirrorPreviewActive = false;
    m_drawState.mirrorPreviewFirstPoint = QVector3D();
    m_drawState.mirrorPreviewSecondPoint = QVector3D();
    m_drawState.rectangularArrayPreviewActive = false;
    m_drawState.rectangularArrayPreviewRows = 1;
    m_drawState.rectangularArrayPreviewColumns = 1;
    m_drawState.rectangularArrayPreviewRowOffset = QVector3D();
    m_drawState.rectangularArrayPreviewColumnOffset = QVector3D();
    m_drawState.circularArrayPreviewActive = false;
    m_drawState.circularArrayPreviewCenter = QVector3D();
    m_drawState.circularArrayPreviewCount = 1;
    m_drawState.circularArrayPreviewTotalAngleDegrees = 0.0;
    m_drawState.circularArrayPreviewRotateItems = true;
    m_drawState.offsetPreviewActive = false;
    m_drawState.offsetPreviewDistance = 0.0;
}

bool CadController::updateCopyPreviewFromCursor()
{
    if (!isParameterInputCommandActive()
        || m_parameterInputSession.command != ParameterInputCommand::Copy
        || m_parameterInputSession.stageIndex != 1)
    {
        return false;
    }

    const QVector3D basePoint = flattenToDrawingPlane(m_parameterInputSession.point1);
    const QVector3D delta = flattenToDrawingPlane(m_drawState.currentPos) - basePoint;
    m_parameterInputSession.doubleValue1 = delta.x();
    m_parameterInputSession.doubleValue2 = delta.y();
    m_drawState.copyPreviewActive = true;
    m_drawState.copyPreviewBasePoint = basePoint;
    m_drawState.copyPreviewDelta = delta;
    return true;
}

bool CadController::finishCopyParameterInput(const QVector3D& delta)
{
    if (delta.lengthSquared() <= 0.000001f || m_editer == nullptr)
    {
        if (m_viewer != nullptr)
        {
            m_viewer->appendCommandMessage(QStringLiteral("复制位移为 0，未创建副本。"));
            m_viewer->refreshCommandPrompt();
            m_viewer->requestViewUpdate();
        }

        m_drawState.editType = EditType::None;
        resetParameterInputSession();
        resetPointDynamicInputSession();
        return true;
    }

    const int requestedCount = m_parameterInputSession.selectedItems.size();
    const bool success = m_editer->copyEntities(m_parameterInputSession.selectedItems, flattenToDrawingPlane(delta));

    if (m_viewer != nullptr)
    {
        m_viewer->appendCommandMessage
        (
            success
                ? QStringLiteral("已复制 %1 个图元，位移为 (%2, %3)。")
                    .arg(requestedCount)
                    .arg(formatDynamicInputValue(delta.x()))
                    .arg(formatDynamicInputValue(delta.y()))
                : QStringLiteral("选中图元复制失败。")
        );
        m_viewer->refreshCommandPrompt();
        m_viewer->requestViewUpdate();
    }

    if (success)
    {
        m_drawState.editType = EditType::None;
        resetParameterInputSession();
        resetPointDynamicInputSession();
    }

    return true;
}

bool CadController::updateRotatePreviewFromCursor()
{
    if (!isParameterInputCommandActive()
        || m_parameterInputSession.command != ParameterInputCommand::Rotate
        || m_parameterInputSession.stageIndex != 1)
    {
        return false;
    }

    const QVector3D basePoint = flattenToDrawingPlane(m_parameterInputSession.point1);

    if (m_parameterInputSession.fieldBuffer.isEmpty())
    {
        m_parameterInputSession.doubleValue1 = rotationAngleFromBaseToPoint(basePoint, m_drawState.currentPos);
    }

    m_drawState.rotatePreviewActive = true;
    m_drawState.rotatePreviewBasePoint = basePoint;
    m_drawState.rotatePreviewAngleDegrees = m_parameterInputSession.doubleValue1;
    return true;
}

bool CadController::finishRotateParameterInput(double angleDegrees)
{
    if (std::abs(angleDegrees) <= 0.000001)
    {
        if (m_viewer != nullptr)
        {
            m_viewer->appendCommandMessage(QStringLiteral("旋转角度为 0，未修改图元。"));
            m_viewer->refreshCommandPrompt();
            m_viewer->requestViewUpdate();
        }

        m_drawState.editType = EditType::None;
        resetParameterInputSession();
        resetPointDynamicInputSession();
        return true;
    }

    if (m_editer == nullptr)
    {
        if (m_viewer != nullptr)
        {
            m_viewer->appendCommandMessage(QStringLiteral("选中图元旋转失败。"));
            m_viewer->refreshCommandPrompt();
            m_viewer->requestViewUpdate();
        }

        return true;
    }

    const int requestedCount = m_parameterInputSession.selectedItems.size();
    const bool success = m_editer->rotateEntities
    (
        m_parameterInputSession.selectedItems,
        flattenToDrawingPlane(m_parameterInputSession.point1),
        angleDegrees
    );

    if (m_viewer != nullptr)
    {
        m_viewer->appendCommandMessage
        (
            success
                ? QStringLiteral("已将 %1 个图元绕基点旋转 %2 度。")
                    .arg(requestedCount)
                    .arg(formatDynamicInputValue(angleDegrees))
                : QStringLiteral("选中图元旋转失败。")
        );
        m_viewer->refreshCommandPrompt();
        m_viewer->requestViewUpdate();
    }

    if (success)
    {
        m_drawState.editType = EditType::None;
        resetParameterInputSession();
        resetPointDynamicInputSession();
    }

    return true;
}

bool CadController::updateScalePreviewFromCursor()
{
    if (!isParameterInputCommandActive()
        || m_parameterInputSession.command != ParameterInputCommand::Scale
        || m_parameterInputSession.stageIndex != 1)
    {
        return false;
    }

    const QVector3D basePoint = flattenToDrawingPlane(m_parameterInputSession.point1);
    const double referenceDistance = std::max(m_parameterInputSession.doubleValue2, 0.001);

    if (m_parameterInputSession.fieldBuffer.isEmpty())
    {
        const double cursorDistance = static_cast<double>((flattenToDrawingPlane(m_drawState.currentPos) - basePoint).length());
        m_parameterInputSession.doubleValue1 = std::max(cursorDistance / referenceDistance, 0.001);
    }

    m_drawState.scalePreviewActive = true;
    m_drawState.scalePreviewBasePoint = basePoint;
    m_drawState.scalePreviewFactor = m_parameterInputSession.doubleValue1;
    return true;
}

bool CadController::finishScaleParameterInput(double scaleFactor)
{
    if (scaleFactor < 0.001 || m_editer == nullptr)
    {
        if (m_viewer != nullptr)
        {
            m_viewer->appendCommandMessage(QStringLiteral("选中图元缩放失败。"));
            m_viewer->refreshCommandPrompt();
            m_viewer->requestViewUpdate();
        }

        return true;
    }

    const int requestedCount = m_parameterInputSession.selectedItems.size();
    const bool success = m_editer->scaleEntities
    (
        m_parameterInputSession.selectedItems,
        flattenToDrawingPlane(m_parameterInputSession.point1),
        scaleFactor
    );

    if (m_viewer != nullptr)
    {
        m_viewer->appendCommandMessage
        (
            success
                ? QStringLiteral("已将 %1 个图元按基点缩放为 %2 倍。")
                    .arg(requestedCount)
                    .arg(formatDynamicInputValue(scaleFactor))
                : QStringLiteral("选中图元缩放失败。")
        );
        m_viewer->refreshCommandPrompt();
        m_viewer->requestViewUpdate();
    }

    if (success)
    {
        m_drawState.editType = EditType::None;
        resetParameterInputSession();
        resetPointDynamicInputSession();
    }

    return true;
}

bool CadController::updateMirrorPreviewFromCursor()
{
    if (!isParameterInputCommandActive()
        || m_parameterInputSession.command != ParameterInputCommand::Mirror
        || m_parameterInputSession.stageIndex < 1)
    {
        return false;
    }

    m_drawState.mirrorPreviewActive = true;
    m_drawState.mirrorPreviewFirstPoint = flattenToDrawingPlane(m_parameterInputSession.point1);
    m_drawState.mirrorPreviewSecondPoint = m_parameterInputSession.stageIndex == 1
        ? flattenToDrawingPlane(m_drawState.currentPos)
        : flattenToDrawingPlane(m_parameterInputSession.point2);
    return true;
}

bool CadController::updateArrayPreviewState()
{
    if (!isParameterInputCommandActive())
    {
        return false;
    }

    if (m_parameterInputSession.command == ParameterInputCommand::RectangularArray)
    {
        m_drawState.rectangularArrayPreviewActive = true;
        m_drawState.rectangularArrayPreviewRows = std::max(m_parameterInputSession.intValue1, 1);
        m_drawState.rectangularArrayPreviewColumns = std::max(m_parameterInputSession.intValue2, 1);
        m_drawState.rectangularArrayPreviewRowOffset = QVector3D(0.0f, static_cast<float>(m_parameterInputSession.doubleValue1), 0.0f);
        m_drawState.rectangularArrayPreviewColumnOffset = QVector3D(static_cast<float>(m_parameterInputSession.doubleValue2), 0.0f, 0.0f);
        return true;
    }

    if (m_parameterInputSession.command == ParameterInputCommand::CircularArray
        && m_parameterInputSession.stageIndex >= 3)
    {
        m_drawState.circularArrayPreviewActive = true;
        m_drawState.circularArrayPreviewCenter = flattenToDrawingPlane(m_parameterInputSession.point1);
        m_drawState.circularArrayPreviewCount = std::max(m_parameterInputSession.intValue1, 1);
        m_drawState.circularArrayPreviewTotalAngleDegrees = m_parameterInputSession.doubleValue1;
        m_drawState.circularArrayPreviewRotateItems = m_parameterInputSession.boolValue;
        return true;
    }

    return false;
}

bool CadController::updateOffsetPreviewFromCursor()
{
    if (!isParameterInputCommandActive()
        || m_parameterInputSession.command != ParameterInputCommand::Offset
        || m_parameterInputSession.stageIndex != 1)
    {
        return false;
    }

    m_drawState.offsetPreviewActive = true;
    m_drawState.offsetPreviewDistance = signedOffsetDistanceForSide
    (
        m_parameterInputSession.primaryItem,
        m_parameterInputSession.doubleValue1,
        m_drawState.currentPos
    );
    return true;
}

bool CadController::finishOffsetParameterInput(double signedDistance)
{
    if (m_editer == nullptr || !m_editer->offsetEntity(m_parameterInputSession.primaryItem, signedDistance))
    {
        if (m_viewer != nullptr)
        {
            m_viewer->appendCommandMessage(QStringLiteral("当前图元不支持偏移，或偏移失败。"));
            m_viewer->refreshCommandPrompt();
            m_viewer->requestViewUpdate();
        }

        return true;
    }

    if (m_viewer != nullptr)
    {
        m_viewer->appendCommandMessage
        (
            QStringLiteral("已创建偏移图元，距离为 %1。")
                .arg(formatDynamicInputValue(signedDistance))
        );
        m_viewer->refreshCommandPrompt();
        m_viewer->requestViewUpdate();
    }

    m_drawState.editType = EditType::None;
    resetParameterInputSession();
    resetPointDynamicInputSession();
    return true;
}

QString CadController::currentPointInputStageKey() const
{
    if (!m_drawState.hasActiveCommand() || !isAwaitingPointInput())
    {
        return QString();
    }

    if (isParameterInputCommandActive())
    {
        switch (m_parameterInputSession.command)
        {
        case ParameterInputCommand::Copy:
            return m_parameterInputSession.stageIndex == 0
                ? QStringLiteral("PARAM_COPY_BASE")
                : QStringLiteral("PARAM_COPY_TARGET");
        case ParameterInputCommand::CircularArray:
            return QStringLiteral("PARAM_ARRAYPOLAR_CENTER");
        case ParameterInputCommand::Mirror:
            return m_parameterInputSession.stageIndex == 0
                ? QStringLiteral("PARAM_MIRROR_FIRST")
                : QStringLiteral("PARAM_MIRROR_SECOND");
        case ParameterInputCommand::Rotate:
            return QStringLiteral("PARAM_ROTATE_BASE");
        case ParameterInputCommand::Scale:
            return QStringLiteral("PARAM_SCALE_BASE");
        case ParameterInputCommand::Offset:
            return QStringLiteral("PARAM_OFFSET_SIDE");
        default:
            break;
        }

        return QStringLiteral("PARAM_POINT");
    }

    if (m_drawState.editType == EditType::Move)
    {
        if (m_drawState.moveSubMode == MoveEditSubMode::AwaitBasePoint)
        {
            return QStringLiteral("MOVE_BASE");
        }

        if (m_drawState.moveSubMode == MoveEditSubMode::AwaitTargetPoint)
        {
            return QStringLiteral("MOVE_TARGET");
        }
    }

    if (m_drawState.editType == EditType::GripEdit)
    {
        if (m_drawState.gripSubMode == GripEditSubMode::AwaitTargetPoint)
        {
            return QStringLiteral("GRIP_TARGET");
        }
    }

    switch (m_drawState.drawType)
    {
    case DrawType::Point:
        return QStringLiteral("POINT_POSITION");
    case DrawType::Line:
        return m_drawState.lineSubMode == LineDrawSubMode::AwaitEndPoint
            ? QStringLiteral("LINE_END")
            : QStringLiteral("LINE_START");
    case DrawType::Xline:
        return m_drawState.lineSubMode == LineDrawSubMode::AwaitEndPoint
            ? QStringLiteral("XLINE_THROUGH")
            : QStringLiteral("XLINE_BASE");
    case DrawType::Rectangle:
        return m_drawState.rectangleSubMode == RectangleDrawSubMode::AwaitSecondCorner
            ? QStringLiteral("RECTANGLE_SECOND")
            : QStringLiteral("RECTANGLE_FIRST");
    case DrawType::Polygon:
        return m_drawState.polygonSubMode == PolygonDrawSubMode::AwaitRadius
            ? QStringLiteral("POLYGON_RADIUS")
            : QStringLiteral("POLYGON_CENTER");
    case DrawType::Circle:
        return m_drawState.circleSubMode == CircleDrawSubMode::AwaitRadius
            ? QStringLiteral("CIRCLE_RADIUS")
            : QStringLiteral("CIRCLE_CENTER");
    case DrawType::Arc:
        switch (m_drawState.arcSubMode)
        {
        case ArcDrawSubMode::AwaitRadius:
            return QStringLiteral("ARC_START");
        case ArcDrawSubMode::AwaitStartAngle:
            return QStringLiteral("ARC_START");
        case ArcDrawSubMode::AwaitEndAngle:
            return QStringLiteral("ARC_END");
        default:
            return QStringLiteral("ARC_CENTER");
        }
    case DrawType::Ellipse:
        switch (m_drawState.ellipseSubMode)
        {
        case EllipseDrawSubMode::AwaitMajorAxis:
            return QStringLiteral("ELLIPSE_MAJOR");
        case EllipseDrawSubMode::AwaitMinorAxis:
            return QStringLiteral("ELLIPSE_MINOR");
        default:
            return QStringLiteral("ELLIPSE_CENTER");
        }
    case DrawType::Polyline:
        return m_drawState.polylineSubMode == PolylineDrawSubMode::AwaitFirstPoint
            ? QStringLiteral("POLYLINE_FIRST")
            : QStringLiteral("POLYLINE_NEXT");
    case DrawType::LWPolyline:
        return m_drawState.lwPolylineSubMode == LWPolylineDrawSubMode::AwaitFirstPoint
            ? QStringLiteral("LWPOLYLINE_FIRST")
            : QStringLiteral("LWPOLYLINE_NEXT");
    default:
        break;
    }

    return QString();
}

void CadController::syncPointDynamicInputSession()
{
    if (!m_drawState.hasActiveCommand() || !isAwaitingPointInput())
    {
        resetPointDynamicInputSession();
        return;
    }

    const QString stageKey = currentPointInputStageKey();

    if (stageKey != m_drawState.dynamicInputStageKey)
    {
        resetPointDynamicInputSession(stageKey);
    }
}

void CadController::resetPointDynamicInputSession(const QString& stageKey)
{
    m_drawState.dynamicInputStageKey = stageKey;
    m_drawState.dynamicInputExpressionMode = false;
    m_drawState.dynamicInputBuffer.clear();
    m_drawState.dynamicInputActiveFieldIndex = 0;
    m_drawState.dynamicInputFieldBuffer.clear();
    m_drawState.dynamicInputXLocked = false;
    m_drawState.dynamicInputYLocked = false;
    m_drawState.dynamicInputXValue = 0.0;
    m_drawState.dynamicInputYValue = 0.0;
}

bool CadController::isPointDynamicFieldModeActive() const
{
    return m_drawState.hasActiveCommand()
        && isAwaitingPointInput()
        && !m_drawState.dynamicInputExpressionMode;
}

bool CadController::hasPendingDynamicKeyboardInput() const
{
    if (isAwaitingParameterFieldInput())
    {
        return !m_parameterInputSession.fieldBuffer.isEmpty();
    }

    if (m_drawState.dynamicInputExpressionMode)
    {
        return !m_drawState.dynamicInputBuffer.isEmpty();
    }

    if (isPointDynamicFieldModeActive())
    {
        return !m_drawState.dynamicInputFieldBuffer.isEmpty();
    }

    return !m_drawState.dynamicInputBuffer.isEmpty();
}

QVector3D CadController::applyPointDynamicFieldOverride(const QVector3D& worldPos, bool includeEditingValue) const
{
    QVector3D point = flattenToDrawingPlane(worldPos);

    if (m_drawState.dynamicInputXLocked)
    {
        point.setX(static_cast<float>(m_drawState.dynamicInputXValue));
    }

    if (m_drawState.dynamicInputYLocked)
    {
        point.setY(static_cast<float>(m_drawState.dynamicInputYValue));
    }

    if (!includeEditingValue || m_drawState.dynamicInputFieldBuffer.isEmpty())
    {
        return point;
    }

    bool parsedOk = false;
    const double parsedValue = m_drawState.dynamicInputFieldBuffer.toDouble(&parsedOk);

    if (!parsedOk)
    {
        return point;
    }

    if (m_drawState.dynamicInputActiveFieldIndex == 0)
    {
        point.setX(static_cast<float>(parsedValue));
    }
    else
    {
        point.setY(static_cast<float>(parsedValue));
    }

    return point;
}

bool CadController::tryParseDynamicFieldBuffer(double& value, QString& errorMessage) const
{
    if (m_drawState.dynamicInputFieldBuffer.trimmed().isEmpty())
    {
        errorMessage = QStringLiteral("请输入数值");
        return false;
    }

    bool parsedOk = false;
    value = m_drawState.dynamicInputFieldBuffer.toDouble(&parsedOk);

    if (!parsedOk)
    {
        errorMessage = QStringLiteral("数值输入无效");
        return false;
    }

    return true;
}

bool CadController::commitActiveDynamicField(QString& errorMessage)
{
    if (!isPointDynamicFieldModeActive())
    {
        return false;
    }

    const int activeFieldIndex = std::clamp(m_drawState.dynamicInputActiveFieldIndex, 0, 1);
    double fieldValue = (activeFieldIndex == 0) ? m_drawState.currentPos.x() : m_drawState.currentPos.y();

    if (!m_drawState.dynamicInputFieldBuffer.isEmpty())
    {
        if (!tryParseDynamicFieldBuffer(fieldValue, errorMessage))
        {
            return false;
        }
    }

    if (activeFieldIndex == 0)
    {
        m_drawState.dynamicInputXLocked = true;
        m_drawState.dynamicInputXValue = fieldValue;
    }
    else
    {
        m_drawState.dynamicInputYLocked = true;
        m_drawState.dynamicInputYValue = fieldValue;
    }

    m_drawState.dynamicInputFieldBuffer.clear();
    return true;
}

bool CadController::handleDynamicFieldTab(int step)
{
    if (!isPointDynamicFieldModeActive())
    {
        return false;
    }

    QString errorMessage;

    if (!commitActiveDynamicField(errorMessage))
    {
        if (m_viewer != nullptr && !errorMessage.isEmpty())
        {
            m_viewer->appendCommandMessage(errorMessage);
            m_viewer->refreshCommandPrompt();
        }

        return true;
    }

    const int normalizedStep = step >= 0 ? 1 : -1;
    int nextFieldIndex = m_drawState.dynamicInputActiveFieldIndex + normalizedStep;

    if (nextFieldIndex < 0)
    {
        nextFieldIndex = 1;
    }
    else if (nextFieldIndex > 1)
    {
        nextFieldIndex = 0;
    }

    m_drawState.dynamicInputActiveFieldIndex = nextFieldIndex;
    m_drawState.currentPos = applyPointDynamicFieldOverride(m_drawState.currentPos, false);

    if (m_viewer != nullptr)
    {
        m_viewer->refreshCommandPrompt();
        m_viewer->requestViewUpdate();
    }

    return true;
}

bool CadController::submitPointDynamicFieldInput()
{
    if (!isPointDynamicFieldModeActive())
    {
        return false;
    }

    QString errorMessage;

    if (!m_drawState.dynamicInputFieldBuffer.isEmpty() && !commitActiveDynamicField(errorMessage))
    {
        if (m_viewer != nullptr)
        {
            m_viewer->appendCommandMessage(errorMessage);
            m_viewer->refreshCommandPrompt();
        }

        return true;
    }

    const QVector3D resolvedPoint = applyPointDynamicFieldOverride(m_drawState.currentPos, false);

    if (commitCommandPoint(resolvedPoint))
    {
        return true;
    }

    if (m_viewer != nullptr)
    {
        m_viewer->appendCommandMessage(QStringLiteral("输入未被当前命令接受"));
        m_viewer->refreshCommandPrompt();
    }

    return true;
}

QString CadController::formatDynamicInputValue(double value)
{
    if (std::abs(value) < 1e-9)
    {
        value = 0.0;
    }

    QString text = QString::number(value, 'f', 6);

    while (text.contains(QLatin1Char('.')) && text.endsWith(QLatin1Char('0')))
    {
        text.chop(1);
    }

    if (text.endsWith(QLatin1Char('.')))
    {
        text.chop(1);
    }

    if (text.isEmpty() || text == QStringLiteral("-0"))
    {
        text = QStringLiteral("0");
    }

    return text;
}

bool CadController::isAwaitingPointInput() const
{
    if (isParameterInputCommandActive())
    {
        return (m_parameterInputSession.command == ParameterInputCommand::Copy
                && (m_parameterInputSession.stageIndex == 0 || m_parameterInputSession.stageIndex == 1))
            || (m_parameterInputSession.command == ParameterInputCommand::CircularArray
                && m_parameterInputSession.stageIndex == 2)
            || (m_parameterInputSession.command == ParameterInputCommand::Rotate
                && m_parameterInputSession.stageIndex == 0)
            || (m_parameterInputSession.command == ParameterInputCommand::Mirror
                && (m_parameterInputSession.stageIndex == 0 || m_parameterInputSession.stageIndex == 1))
            || (m_parameterInputSession.command == ParameterInputCommand::Scale
                && m_parameterInputSession.stageIndex == 0)
            || (m_parameterInputSession.command == ParameterInputCommand::Offset
                && m_parameterInputSession.stageIndex == 1);
    }

    if (m_drawState.editType == EditType::Move)
    {
        return m_drawState.moveSubMode == MoveEditSubMode::AwaitBasePoint
            || m_drawState.moveSubMode == MoveEditSubMode::AwaitTargetPoint;
    }

    if (m_drawState.editType == EditType::GripEdit)
    {
        return m_drawState.gripSubMode == GripEditSubMode::AwaitTargetPoint;
    }

    if (!m_drawState.isDrawing)
    {
        return false;
    }

    switch (m_drawState.drawType)
    {
    case DrawType::Point:
        return m_drawState.pointSubMode == PointDrawSubMode::AwaitPosition;
    case DrawType::Line:
    case DrawType::Xline:
        return m_drawState.lineSubMode == LineDrawSubMode::AwaitStartPoint
            || m_drawState.lineSubMode == LineDrawSubMode::AwaitEndPoint;
    case DrawType::Rectangle:
        return m_drawState.rectangleSubMode == RectangleDrawSubMode::AwaitFirstCorner
            || m_drawState.rectangleSubMode == RectangleDrawSubMode::AwaitSecondCorner;
    case DrawType::Polygon:
        return m_drawState.polygonSubMode == PolygonDrawSubMode::AwaitCenter
            || m_drawState.polygonSubMode == PolygonDrawSubMode::AwaitRadius;
    case DrawType::Circle:
        return m_drawState.circleSubMode == CircleDrawSubMode::AwaitCenter
            || m_drawState.circleSubMode == CircleDrawSubMode::AwaitRadius;
    case DrawType::Arc:
        return m_drawState.arcSubMode == ArcDrawSubMode::AwaitCenter
            || m_drawState.arcSubMode == ArcDrawSubMode::AwaitRadius
            || m_drawState.arcSubMode == ArcDrawSubMode::AwaitStartAngle
            || m_drawState.arcSubMode == ArcDrawSubMode::AwaitEndAngle;
    case DrawType::Ellipse:
        return m_drawState.ellipseSubMode == EllipseDrawSubMode::AwaitCenter
            || m_drawState.ellipseSubMode == EllipseDrawSubMode::AwaitMajorAxis
            || m_drawState.ellipseSubMode == EllipseDrawSubMode::AwaitMinorAxis;
    case DrawType::Polyline:
        return m_drawState.polylineSubMode == PolylineDrawSubMode::AwaitFirstPoint
            || m_drawState.polylineSubMode == PolylineDrawSubMode::AwaitLineEndPoint
            || m_drawState.polylineSubMode == PolylineDrawSubMode::AwaitArcEndPoint;
    case DrawType::LWPolyline:
        return m_drawState.lwPolylineSubMode == LWPolylineDrawSubMode::AwaitFirstPoint
            || m_drawState.lwPolylineSubMode == LWPolylineDrawSubMode::AwaitLineEndPoint
            || m_drawState.lwPolylineSubMode == LWPolylineDrawSubMode::AwaitArcEndPoint;
    default:
        break;
    }

    return false;
}

QVector3D CadController::dynamicInputReferencePoint() const
{
    if (isParameterInputCommandActive()
        && m_parameterInputSession.command == ParameterInputCommand::Copy
        && m_parameterInputSession.stageIndex == 1)
    {
        return flattenToDrawingPlane(m_parameterInputSession.point1);
    }

    if (isParameterInputCommandActive()
        && m_parameterInputSession.command == ParameterInputCommand::Mirror
        && m_parameterInputSession.stageIndex == 1)
    {
        return flattenToDrawingPlane(m_parameterInputSession.point1);
    }

    if (!m_drawState.commandPoints.isEmpty())
    {
        return flattenToDrawingPlane(m_drawState.commandPoints.back());
    }

    return flattenToDrawingPlane(m_drawState.currentPos);
}

QVector3D CadController::applyOrthoConstraint(const QVector3D& worldPos) const
{
    constexpr double kPolarAngleIncrementDegrees = 15.0;
    constexpr double kDegreesPerRadian = 180.0 / 3.14159265358979323846;
    constexpr double kRadiansPerDegree = 3.14159265358979323846 / 180.0;
    const QVector3D planarPoint = flattenToDrawingPlane(worldPos);
    const bool hasConstraintReference = !m_drawState.commandPoints.isEmpty()
        || (isParameterInputCommandActive()
            && m_parameterInputSession.command == ParameterInputCommand::Copy
            && m_parameterInputSession.stageIndex == 1)
        || (isParameterInputCommandActive()
            && m_parameterInputSession.command == ParameterInputCommand::Mirror
            && m_parameterInputSession.stageIndex == 1);

    if (!m_drawState.hasActiveCommand()
        || !isAwaitingPointInput()
        || !hasConstraintReference)
    {
        return planarPoint;
    }

    const QVector3D basePoint = dynamicInputReferencePoint();
    const QVector3D delta = planarPoint - basePoint;

    if (m_drawState.orthoEnabled)
    {
        if (std::abs(delta.x()) >= std::abs(delta.y()))
        {
            return QVector3D(planarPoint.x(), basePoint.y(), 0.0f);
        }

        return QVector3D(basePoint.x(), planarPoint.y(), 0.0f);
    }

    if (!m_drawState.polarTrackingEnabled)
    {
        return planarPoint;
    }

    const double distance = std::hypot(static_cast<double>(delta.x()), static_cast<double>(delta.y()));

    if (distance <= 1.0e-9)
    {
        return planarPoint;
    }

    const double angleDegrees = std::atan2(static_cast<double>(delta.y()), static_cast<double>(delta.x())) * kDegreesPerRadian;
    const double constrainedDegrees = std::round(angleDegrees / kPolarAngleIncrementDegrees) * kPolarAngleIncrementDegrees;
    const double constrainedRadians = constrainedDegrees * kRadiansPerDegree;
    return QVector3D
    (
        static_cast<float>(basePoint.x() + distance * std::cos(constrainedRadians)),
        static_cast<float>(basePoint.y() + distance * std::sin(constrainedRadians)),
        0.0f
    );
}

bool CadController::tryResolveDynamicInputPoint(const QString& inputText, QVector3D& worldPoint, QString& errorMessage) const
{
    QString normalizedInput = inputText;
    normalizedInput.remove(QLatin1Char(' '));
    normalizedInput.remove(QLatin1Char('\t'));

    if (normalizedInput.isEmpty())
    {
        errorMessage = QStringLiteral("请输入坐标值");
        return false;
    }

    const QVector3D referencePoint = dynamicInputReferencePoint();
    const int polarSeparator = normalizedInput.indexOf(QLatin1Char('<'));

    if (polarSeparator >= 0)
    {
        const QString distanceText = normalizedInput.left(polarSeparator).trimmed();
        const QString angleText = normalizedInput.mid(polarSeparator + 1).trimmed();
        const QString normalizedDistanceText = distanceText.startsWith(QLatin1Char('@'))
            ? distanceText.mid(1)
            : distanceText;

        bool distanceOk = false;
        bool angleOk = false;
        const double distance = normalizedDistanceText.toDouble(&distanceOk);
        const double angleDegrees = angleText.toDouble(&angleOk);

        if (!distanceOk || !angleOk)
        {
            errorMessage = QStringLiteral("极坐标输入格式无效，应为 距离<角度，例如 100<30");
            return false;
        }

        constexpr double kRadiansPerDegree = 3.14159265358979323846 / 180.0;
        const double radians = angleDegrees * kRadiansPerDegree;
        worldPoint = QVector3D
        (
            static_cast<float>(referencePoint.x() + distance * std::cos(radians)),
            static_cast<float>(referencePoint.y() + distance * std::sin(radians)),
            0.0f
        );
        worldPoint = applyOrthoConstraint(worldPoint);
        return true;
    }

    const bool relative = normalizedInput.startsWith(QLatin1Char('@'));
    const QString coordinateText = relative ? normalizedInput.mid(1) : normalizedInput;
    double firstValue = 0.0;
    double secondValue = 0.0;

    if (!tryParseCoordinatePair(coordinateText, firstValue, secondValue))
    {
        errorMessage = QStringLiteral("坐标输入格式无效，应为 x,y 或 @dx,dy");
        return false;
    }

    if (relative)
    {
        worldPoint = QVector3D
        (
            static_cast<float>(referencePoint.x() + firstValue),
            static_cast<float>(referencePoint.y() + secondValue),
            0.0f
        );
    }
    else
    {
        worldPoint = QVector3D(static_cast<float>(firstValue), static_cast<float>(secondValue), 0.0f);
    }

    worldPoint = applyOrthoConstraint(worldPoint);
    return true;
}

bool CadController::commitCommandPoint(const QVector3D& worldPos)
{
    if (!m_drawState.hasActiveCommand())
    {
        return false;
    }

    syncPointDynamicInputSession();

    QVector3D constrainedWorldPos = applyOrthoConstraint(worldPos);

    if (isPointDynamicFieldModeActive())
    {
        constrainedWorldPos = applyPointDynamicFieldOverride(constrainedWorldPos, true);
    }

    m_drawState.currentPos = constrainedWorldPos;

    if (isParameterInputCommandActive())
    {
        return commitParameterInputPoint(constrainedWorldPos);
    }

    if (m_editer == nullptr)
    {
        return false;
    }

    const DrawStateMachine previousState = m_drawState;
    handleLeftPressInCommand(constrainedWorldPos);

    if (!m_editer->handleLeftPress(previousState, m_drawState, constrainedWorldPos))
    {
        return false;
    }

    resetPointDynamicInputSession(currentPointInputStageKey());

    if (m_drawState.hasActiveCommand() && isAwaitingPointInput())
    {
        syncCurrentPosWithCursor();
    }

    if (m_viewer != nullptr)
    {
        if (previousState.editType == EditType::Move && m_drawState.editType == EditType::None)
        {
            m_viewer->appendCommandMessage(QStringLiteral("移动完成"));
        }
        else if (previousState.editType == EditType::GripEdit && m_drawState.editType == EditType::None)
        {
            m_viewer->appendCommandMessage(QStringLiteral("控制点编辑完成"));
        }
        else if (previousState.drawType == DrawType::Point)
        {
            m_viewer->appendCommandMessage(QStringLiteral("已创建点图元"));
        }
        else if (previousState.drawType == DrawType::Rectangle
            && previousState.rectangleSubMode == RectangleDrawSubMode::AwaitSecondCorner
            && m_drawState.rectangleSubMode == RectangleDrawSubMode::AwaitFirstCorner)
        {
            m_viewer->appendCommandMessage(QStringLiteral("已创建矩形图元"));
        }
        else if (previousState.drawType == DrawType::Polygon
            && previousState.polygonSubMode == PolygonDrawSubMode::AwaitRadius
            && m_drawState.polygonSubMode == PolygonDrawSubMode::AwaitCenter)
        {
            m_viewer->appendCommandMessage
            (
                QStringLiteral("已创建多边形图元（%1 边，%2）")
                .arg(previousState.polygonSideCount)
                .arg(polygonConstructionModeText(previousState.polygonCircumscribedAboutCircle))
            );
        }
        else if (previousState.drawType == DrawType::Circle
            && previousState.circleSubMode == CircleDrawSubMode::AwaitRadius
            && m_drawState.circleSubMode == CircleDrawSubMode::AwaitCenter)
        {
            m_viewer->appendCommandMessage(QStringLiteral("已创建圆图元"));
        }
        else if (previousState.drawType == DrawType::Arc
            && previousState.arcSubMode == ArcDrawSubMode::AwaitEndAngle
            && m_drawState.arcSubMode == ArcDrawSubMode::AwaitCenter)
        {
            m_viewer->appendCommandMessage(QStringLiteral("已创建圆弧图元"));
        }
        else if (previousState.drawType == DrawType::Ellipse
            && previousState.ellipseSubMode == EllipseDrawSubMode::AwaitMinorAxis
            && m_drawState.ellipseSubMode == EllipseDrawSubMode::AwaitCenter)
        {
            m_viewer->appendCommandMessage(QStringLiteral("已创建椭圆图元"));
        }
        else if (previousState.drawType == DrawType::Line
            && previousState.lineSubMode == LineDrawSubMode::AwaitEndPoint
            && m_drawState.lineSubMode == LineDrawSubMode::AwaitEndPoint)
        {
            m_viewer->appendCommandMessage(QStringLiteral("已创建直线图元"));
        }
        else if (previousState.drawType == DrawType::Xline
            && previousState.lineSubMode == LineDrawSubMode::AwaitEndPoint
            && m_drawState.lineSubMode == LineDrawSubMode::AwaitStartPoint)
        {
            m_viewer->appendCommandMessage(QStringLiteral("已创建构造线图元"));
        }

        m_viewer->refreshCommandPrompt();
        m_viewer->requestViewUpdate();
    }

    return true;
}

bool CadController::submitDynamicInputBuffer()
{
    syncPointDynamicInputSession();

    if (m_drawState.dynamicInputBuffer.isEmpty())
    {
        return false;
    }

    if (!isAwaitingPointInput())
    {
        if (m_viewer != nullptr)
        {
            m_viewer->appendCommandMessage(QStringLiteral("当前命令阶段不接受坐标输入"));
            m_viewer->refreshCommandPrompt();
        }

        return true;
    }

    QVector3D parsedPoint;
    QString errorMessage;

    if (!tryResolveDynamicInputPoint(m_drawState.dynamicInputBuffer, parsedPoint, errorMessage))
    {
        if (m_viewer != nullptr)
        {
            m_viewer->appendCommandMessage(errorMessage);
            m_viewer->refreshCommandPrompt();
        }

        return true;
    }

    if (commitCommandPoint(parsedPoint))
    {
        return true;
    }

    if (m_viewer != nullptr)
    {
        m_viewer->appendCommandMessage(QStringLiteral("输入未被当前命令接受"));
        m_viewer->refreshCommandPrompt();
    }

    return true;
}

bool CadController::submitParameterInputField()
{
    if (!isAwaitingParameterFieldInput())
    {
        return false;
    }

    const QString trimmedInput = m_parameterInputSession.fieldBuffer.trimmed();

    auto commitIntValue = [&](int& targetValue, int minValue, int maxValue, QString& errorMessage) -> bool
    {
        if (trimmedInput.isEmpty())
        {
            return true;
        }

        bool parsedOk = false;
        const int parsedValue = trimmedInput.toInt(&parsedOk);

        if (!parsedOk || parsedValue < minValue || parsedValue > maxValue)
        {
            errorMessage = QStringLiteral("请输入 %1 到 %2 之间的整数").arg(minValue).arg(maxValue);
            return false;
        }

        targetValue = parsedValue;
        return true;
    };

    auto commitDoubleValue = [&](double& targetValue, double minValue, bool allowEqualMin, QString& errorMessage) -> bool
    {
        if (trimmedInput.isEmpty())
        {
            return true;
        }

        bool parsedOk = false;
        const double parsedValue = trimmedInput.toDouble(&parsedOk);

        if (!parsedOk || parsedValue < minValue || (!allowEqualMin && parsedValue <= minValue))
        {
            errorMessage = allowEqualMin
                ? QStringLiteral("请输入不小于 %1 的数值").arg(formatDynamicInputValue(minValue))
                : QStringLiteral("请输入大于 %1 的数值").arg(formatDynamicInputValue(minValue));
            return false;
        }

        targetValue = parsedValue;
        return true;
    };

    auto commitChoiceValue = [&](bool& targetValue, const QStringList& falseTokens, const QStringList& trueTokens, QString& errorMessage) -> bool
    {
        if (trimmedInput.isEmpty())
        {
            return true;
        }

        const QString normalized = trimmedInput.trimmed().toLower();

        for (const QString& token : falseTokens)
        {
            if (normalized == token.toLower())
            {
                targetValue = false;
                return true;
            }
        }

        for (const QString& token : trueTokens)
        {
            if (normalized == token.toLower())
            {
                targetValue = true;
                return true;
            }
        }

        errorMessage = QStringLiteral("输入值无效");
        return false;
    };

    auto finishSession = [&](bool success, const QString& successMessage, const QString& errorMessage) -> bool
    {
        if (m_viewer != nullptr)
        {
            if (success)
            {
                m_viewer->appendCommandMessage(successMessage);
            }
            else if (!errorMessage.isEmpty())
            {
                m_viewer->appendCommandMessage(errorMessage);
            }

            m_viewer->refreshCommandPrompt();
            m_viewer->requestViewUpdate();
        }

        if (success)
        {
            m_drawState.editType = EditType::None;
            resetParameterInputSession();
            resetPointDynamicInputSession();
        }

        return true;
    };

    QString errorMessage;

    switch (m_parameterInputSession.command)
    {
    case ParameterInputCommand::Polygon:
        if (m_parameterInputSession.stageIndex == 0)
        {
            if (!commitIntValue(m_parameterInputSession.intValue1, 3, 1024, errorMessage))
            {
                break;
            }

            m_parameterInputSession.stageIndex = 1;
            m_parameterInputSession.fieldBuffer.clear();
            updateArrayPreviewState();
            if (m_viewer != nullptr)
            {
                m_viewer->refreshCommandPrompt();
                m_viewer->requestViewUpdate();
            }
            return true;
        }

        if (!commitChoiceValue
        (
            m_parameterInputSession.boolValue,
            { QStringLiteral("0"), QStringLiteral("内"), QStringLiteral("内切"), QStringLiteral("内切于圆"), QStringLiteral("inscribed") },
            { QStringLiteral("1"), QStringLiteral("外"), QStringLiteral("外切"), QStringLiteral("外切于圆"), QStringLiteral("circumscribed") },
            errorMessage
        ))
        {
            break;
        }

        m_drawState.polygonSideCount = m_parameterInputSession.intValue1;
        m_drawState.polygonCircumscribedAboutCircle = m_parameterInputSession.boolValue;
        {
            const QColor drawingColor = m_parameterInputSession.drawingColor;
            resetParameterInputSession();
            if (m_editer != nullptr)
            {
                m_editer->cancelTransientCommand();
            }

            m_drawState.isDrawing = true;
            m_drawState.drawType = DrawType::Polygon;
            m_drawState.drawingColor = drawingColor;
            m_drawState.editType = EditType::None;
            m_drawState.commandPoints.clear();
            m_drawState.commandBulges.clear();
            m_drawState.polylineArcMode = false;
            m_drawState.lwPolylineArcMode = false;
            clearDynamicCommandMode();
            resetPointDynamicInputSession();
            resetSubModes();
            preparePrimitiveSubMode();
            resetPointDynamicInputSession(currentPointInputStageKey());

            if (m_viewer != nullptr)
            {
                m_viewer->appendCommandMessage
                (
                    QStringLiteral("已进入多边形命令（%1 边，%2）")
                    .arg(m_drawState.polygonSideCount)
                    .arg(polygonConstructionModeText(m_drawState.polygonCircumscribedAboutCircle))
                );
                m_viewer->refreshCommandPrompt();
                m_viewer->requestViewUpdate();
            }
        }
        return true;

    case ParameterInputCommand::Copy:
        if (m_parameterInputSession.stageIndex == 0)
        {
            if (!commitDoubleValue(m_parameterInputSession.doubleValue1, -1000000.0, true, errorMessage))
            {
                break;
            }

            m_parameterInputSession.stageIndex = 1;
            m_parameterInputSession.fieldBuffer.clear();
            if (m_viewer != nullptr)
            {
                m_viewer->refreshCommandPrompt();
                m_viewer->requestViewUpdate();
            }
            return true;
        }

        if (!commitDoubleValue(m_parameterInputSession.doubleValue2, -1000000.0, true, errorMessage))
        {
            break;
        }

        {
            int copiedCount = 0;
            const QVector3D delta
            (
                static_cast<float>(m_parameterInputSession.doubleValue1),
                static_cast<float>(m_parameterInputSession.doubleValue2),
                0.0f
            );

            for (CadItem* item : m_parameterInputSession.selectedItems)
            {
                if (item != nullptr && m_editer != nullptr && m_editer->copyEntity(item, delta))
                {
                    ++copiedCount;
                }
            }

            return finishSession
            (
                copiedCount > 0,
                QStringLiteral("已复制 %1 个图元，偏移量为 (%2, %3)。")
                    .arg(copiedCount)
                    .arg(formatDynamicInputValue(m_parameterInputSession.doubleValue1))
                    .arg(formatDynamicInputValue(m_parameterInputSession.doubleValue2)),
                QStringLiteral("选中图元复制失败。")
            );
        }

    case ParameterInputCommand::Rotate:
        if (!commitDoubleValue(m_parameterInputSession.doubleValue1, -3600.0, true, errorMessage))
        {
            break;
        }

        return finishRotateParameterInput(m_parameterInputSession.doubleValue1);

    case ParameterInputCommand::Scale:
        if (!commitDoubleValue(m_parameterInputSession.doubleValue1, 0.001, true, errorMessage))
        {
            break;
        }

        return finishScaleParameterInput(m_parameterInputSession.doubleValue1);

    case ParameterInputCommand::RectangularArray:
        switch (m_parameterInputSession.stageIndex)
        {
        case 0:
            if (!commitIntValue(m_parameterInputSession.intValue1, 1, 999, errorMessage))
            {
                break;
            }
            m_parameterInputSession.stageIndex = 1;
            m_parameterInputSession.fieldBuffer.clear();
            if (m_viewer != nullptr)
            {
                m_viewer->refreshCommandPrompt();
                m_viewer->requestViewUpdate();
            }
            return true;
        case 1:
            if (!commitIntValue(m_parameterInputSession.intValue2, 1, 999, errorMessage))
            {
                break;
            }
            if (m_parameterInputSession.intValue1 == 1 && m_parameterInputSession.intValue2 == 1)
            {
                errorMessage = QStringLiteral("行数和列数不能同时为 1。");
                break;
            }
            m_parameterInputSession.stageIndex = 2;
            m_parameterInputSession.fieldBuffer.clear();
            updateArrayPreviewState();
            if (m_viewer != nullptr)
            {
                m_viewer->refreshCommandPrompt();
                m_viewer->requestViewUpdate();
            }
            return true;
        case 2:
            if (!commitDoubleValue(m_parameterInputSession.doubleValue1, -1000000.0, true, errorMessage))
            {
                break;
            }
            m_parameterInputSession.stageIndex = 3;
            m_parameterInputSession.fieldBuffer.clear();
            updateArrayPreviewState();
            if (m_viewer != nullptr)
            {
                m_viewer->refreshCommandPrompt();
                m_viewer->requestViewUpdate();
            }
            return true;
        default:
            if (!commitDoubleValue(m_parameterInputSession.doubleValue2, -1000000.0, true, errorMessage))
            {
                break;
            }

            updateArrayPreviewState();

            {
                const QVector3D rowOffset(0.0f, static_cast<float>(m_parameterInputSession.doubleValue1), 0.0f);
                const QVector3D columnOffset(static_cast<float>(m_parameterInputSession.doubleValue2), 0.0f, 0.0f);

                return finishSession
                (
                    m_editer != nullptr && m_editer->rectangularArrayEntities
                    (
                        m_parameterInputSession.selectedItems,
                        m_parameterInputSession.intValue1,
                        m_parameterInputSession.intValue2,
                        rowOffset,
                        columnOffset
                    ),
                    QStringLiteral("已对 %1 个图元执行 %2 x %3 矩形阵列。")
                        .arg(m_parameterInputSession.selectedItems.size())
                        .arg(m_parameterInputSession.intValue1)
                        .arg(m_parameterInputSession.intValue2),
                    QStringLiteral("选中图元阵列失败。")
                );
            }
        }
        break;

    case ParameterInputCommand::CircularArray:
        if (m_parameterInputSession.stageIndex == 0)
        {
            if (!commitIntValue(m_parameterInputSession.intValue1, 2, 1024, errorMessage))
            {
                break;
            }

            m_parameterInputSession.stageIndex = 1;
            m_parameterInputSession.fieldBuffer.clear();
            updateArrayPreviewState();
            if (m_viewer != nullptr)
            {
                m_viewer->refreshCommandPrompt();
                m_viewer->requestViewUpdate();
            }
            return true;
        }

        if (m_parameterInputSession.stageIndex == 1)
        {
            if (!commitDoubleValue(m_parameterInputSession.doubleValue1, -3600.0, true, errorMessage))
            {
                break;
            }

            m_parameterInputSession.stageIndex = 2;
            m_parameterInputSession.fieldBuffer.clear();
            resetPointDynamicInputSession(currentPointInputStageKey());
            updateArrayPreviewState();
            if (m_viewer != nullptr)
            {
                syncCurrentPosWithCursor();
                m_viewer->refreshCommandPrompt();
                m_viewer->requestViewUpdate();
            }
            return true;
        }

        if (!commitChoiceValue
        (
            m_parameterInputSession.boolValue,
            { QStringLiteral("0"), QStringLiteral("保持"), QStringLiteral("保持原方向"), QStringLiteral("keep") },
            { QStringLiteral("1"), QStringLiteral("旋转"), QStringLiteral("旋转副本方向"), QStringLiteral("rotate") },
            errorMessage
        ))
        {
            break;
        }

        updateArrayPreviewState();

        if (m_editer == nullptr || !m_editer->polarArrayEntities
        (
            m_parameterInputSession.selectedItems,
            m_parameterInputSession.point1,
            m_parameterInputSession.intValue1,
            m_parameterInputSession.doubleValue1,
            m_parameterInputSession.boolValue
        ))
        {
            return finishSession(false, QString(), QStringLiteral("选中图元环形阵列失败。"));
        }

        return finishSession
        (
            true,
            QStringLiteral("已对 %1 个图元执行 %2 项环形阵列。")
                .arg(m_parameterInputSession.selectedItems.size())
                .arg(m_parameterInputSession.intValue1),
            QString()
        );

    case ParameterInputCommand::Mirror:
        if (!commitChoiceValue
        (
            m_parameterInputSession.boolValue,
            { QStringLiteral("0"), QStringLiteral("否"), QStringLiteral("不"), QStringLiteral("no") },
            { QStringLiteral("1"), QStringLiteral("是"), QStringLiteral("删除"), QStringLiteral("yes") },
            errorMessage
        ))
        {
            break;
        }

        if (m_editer == nullptr || !m_editer->mirrorEntities
        (
            m_parameterInputSession.selectedItems,
            m_parameterInputSession.point1,
            m_parameterInputSession.point2,
            m_parameterInputSession.boolValue
        ))
        {
            return finishSession(false, QString(), QStringLiteral("选中图元镜像失败。"));
        }

        return finishSession(true, QStringLiteral("已执行镜像操作。"), QString());

    case ParameterInputCommand::Offset:
        if (!commitDoubleValue(m_parameterInputSession.doubleValue1, -1000000.0, true, errorMessage))
        {
            break;
        }

        if (std::abs(m_parameterInputSession.doubleValue1) <= 0.000001)
        {
            errorMessage = QStringLiteral("偏移距离不能为 0。");
            break;
        }

        m_parameterInputSession.stageIndex = 1;
        m_parameterInputSession.fieldBuffer.clear();
        resetPointDynamicInputSession(currentPointInputStageKey());
        updateOffsetPreviewFromCursor();
        if (m_viewer != nullptr)
        {
            syncCurrentPosWithCursor();
            m_viewer->refreshCommandPrompt();
            m_viewer->requestViewUpdate();
        }
        return true;

    case ParameterInputCommand::Trim:
        if (!commitChoiceValue
        (
            m_parameterInputSession.boolValue,
            { QStringLiteral("1"), QStringLiteral("终"), QStringLiteral("终点"), QStringLiteral("终点端"), QStringLiteral("end") },
            { QStringLiteral("0"), QStringLiteral("起"), QStringLiteral("起点"), QStringLiteral("起点端"), QStringLiteral("start") },
            errorMessage
        ))
        {
            break;
        }

        if (m_editer == nullptr || !m_editer->trimEntity(m_parameterInputSession.secondaryItem, m_parameterInputSession.primaryItem, m_parameterInputSession.boolValue))
        {
            return finishSession(false, QString(), QStringLiteral("当前仅支持将直线修剪到直线、圆或圆弧边界。"));
        }

        return finishSession(true, QStringLiteral("已执行修剪操作。"), QString());

    case ParameterInputCommand::Extend:
        if (!commitChoiceValue
        (
            m_parameterInputSession.boolValue,
            { QStringLiteral("1"), QStringLiteral("终"), QStringLiteral("终点"), QStringLiteral("终点端"), QStringLiteral("end") },
            { QStringLiteral("0"), QStringLiteral("起"), QStringLiteral("起点"), QStringLiteral("起点端"), QStringLiteral("start") },
            errorMessage
        ))
        {
            break;
        }

        if (m_editer == nullptr || !m_editer->extendEntity(m_parameterInputSession.secondaryItem, m_parameterInputSession.primaryItem, m_parameterInputSession.boolValue))
        {
            return finishSession(false, QString(), QStringLiteral("当前仅支持将直线延申到直线、圆或圆弧边界。"));
        }

        return finishSession(true, QStringLiteral("已执行延申操作。"), QString());

    case ParameterInputCommand::Fillet:
        if (!commitDoubleValue(m_parameterInputSession.doubleValue1, 0.001, true, errorMessage))
        {
            break;
        }

        if (m_editer == nullptr || !m_editer->filletEntities(m_parameterInputSession.primaryItem, m_parameterInputSession.secondaryItem, m_parameterInputSession.doubleValue1))
        {
            return finishSession(false, QString(), QStringLiteral("当前仅支持 2 条直线的圆角。"));
        }

        return finishSession
        (
            true,
            QStringLiteral("已执行圆角操作，半径 %1。").arg(formatDynamicInputValue(m_parameterInputSession.doubleValue1)),
            QString()
        );

    case ParameterInputCommand::Chamfer:
        if (m_parameterInputSession.stageIndex == 0)
        {
            if (!commitDoubleValue(m_parameterInputSession.doubleValue1, 0.0, true, errorMessage))
            {
                break;
            }
            m_parameterInputSession.stageIndex = 1;
            m_parameterInputSession.fieldBuffer.clear();
            if (m_viewer != nullptr)
            {
                m_viewer->refreshCommandPrompt();
                m_viewer->requestViewUpdate();
            }
            return true;
        }

        if (!commitDoubleValue(m_parameterInputSession.doubleValue2, 0.0, true, errorMessage))
        {
            break;
        }

        if (m_editer == nullptr || !m_editer->chamferEntities
        (
            m_parameterInputSession.primaryItem,
            m_parameterInputSession.secondaryItem,
            m_parameterInputSession.doubleValue1,
            m_parameterInputSession.doubleValue2
        ))
        {
            return finishSession(false, QString(), QStringLiteral("当前仅支持 2 条直线的倒角。"));
        }

        return finishSession
        (
            true,
            QStringLiteral("已执行倒角操作，距离为 %1 / %2。")
                .arg(formatDynamicInputValue(m_parameterInputSession.doubleValue1))
                .arg(formatDynamicInputValue(m_parameterInputSession.doubleValue2)),
            QString()
        );

    default:
        break;
    }

    if (m_viewer != nullptr)
    {
        if (!errorMessage.isEmpty())
        {
            m_viewer->appendCommandMessage(errorMessage);
        }
        m_viewer->refreshCommandPrompt();
        m_viewer->requestViewUpdate();
    }

    return true;
}

bool CadController::commitParameterInputPoint(const QVector3D& worldPos)
{
    if (!isParameterInputCommandActive())
    {
        return false;
    }

    switch (m_parameterInputSession.command)
    {
    case ParameterInputCommand::Copy:
        if (m_parameterInputSession.stageIndex == 0)
        {
            m_parameterInputSession.point1 = flattenToDrawingPlane(worldPos);
            m_parameterInputSession.stageIndex = 1;
            m_parameterInputSession.fieldBuffer.clear();
            updateCopyPreviewFromCursor();
            break;
        }

        {
            const QVector3D delta = flattenToDrawingPlane(worldPos) - flattenToDrawingPlane(m_parameterInputSession.point1);
            return finishCopyParameterInput(delta);
        }
    case ParameterInputCommand::CircularArray:
        m_parameterInputSession.point1 = flattenToDrawingPlane(worldPos);
        m_parameterInputSession.stageIndex = 3;
        updateArrayPreviewState();
        break;
    case ParameterInputCommand::Rotate:
        if (m_parameterInputSession.stageIndex == 0)
        {
            m_parameterInputSession.point1 = flattenToDrawingPlane(worldPos);
            m_parameterInputSession.doubleValue1 = rotationAngleFromBaseToPoint(m_parameterInputSession.point1, m_drawState.currentPos);
            m_parameterInputSession.stageIndex = 1;
            m_parameterInputSession.fieldBuffer.clear();
            updateRotatePreviewFromCursor();
            break;
        }

        updateRotatePreviewFromCursor();
        return finishRotateParameterInput(m_parameterInputSession.doubleValue1);
    case ParameterInputCommand::Scale:
        if (m_parameterInputSession.stageIndex == 0)
        {
            m_parameterInputSession.point1 = flattenToDrawingPlane(worldPos);
            m_parameterInputSession.doubleValue1 = 1.0;
            m_parameterInputSession.doubleValue2 = scaleReferenceDistance
            (
                m_parameterInputSession.selectedItems,
                m_parameterInputSession.point1
            );
            m_parameterInputSession.stageIndex = 1;
            m_parameterInputSession.fieldBuffer.clear();
            updateScalePreviewFromCursor();
            break;
        }

        updateScalePreviewFromCursor();
        return finishScaleParameterInput(m_parameterInputSession.doubleValue1);
    case ParameterInputCommand::Mirror:
        if (m_parameterInputSession.stageIndex == 0)
        {
            m_parameterInputSession.point1 = flattenToDrawingPlane(worldPos);
            m_parameterInputSession.stageIndex = 1;
            updateMirrorPreviewFromCursor();
        }
        else
        {
            m_parameterInputSession.point2 = flattenToDrawingPlane(worldPos);
            m_parameterInputSession.stageIndex = 2;
            updateMirrorPreviewFromCursor();
        }
        break;
    case ParameterInputCommand::Offset:
        if (m_parameterInputSession.stageIndex == 1)
        {
            const double signedDistance = signedOffsetDistanceForSide
            (
                m_parameterInputSession.primaryItem,
                m_parameterInputSession.doubleValue1,
                worldPos
            );
            m_drawState.offsetPreviewDistance = signedDistance;
            return finishOffsetParameterInput(signedDistance);
        }
        return false;
    default:
        return false;
    }

    resetPointDynamicInputSession(currentPointInputStageKey());

    if (m_viewer != nullptr)
    {
        syncCurrentPosWithCursor();
        m_viewer->refreshCommandPrompt();
        m_viewer->requestViewUpdate();
    }

    return true;
}

bool CadController::confirmActiveCommand()
{
    if (!m_drawState.hasActiveCommand())
    {
        return false;
    }

    if (isParameterInputCommandActive())
    {
        syncPointDynamicInputSession();
        syncCurrentPosWithCursor();

        if (isAwaitingPointInput())
        {
            if (isPointDynamicFieldModeActive())
            {
                return submitPointDynamicFieldInput();
            }

            if (!m_drawState.dynamicInputBuffer.isEmpty())
            {
                return submitDynamicInputBuffer();
            }

            if (!commitCommandPoint(m_drawState.currentPos) && m_viewer != nullptr)
            {
                m_viewer->refreshCommandPrompt();
            }

            return true;
        }

        return submitParameterInputField();
    }

    syncPointDynamicInputSession();
    syncCurrentPosWithCursor();

    if (isPointDynamicFieldModeActive())
    {
        if ((m_drawState.drawType == DrawType::Polyline || m_drawState.drawType == DrawType::LWPolyline)
            && m_editer != nullptr
            && !hasPendingDynamicKeyboardInput()
            && !m_drawState.dynamicInputXLocked
            && !m_drawState.dynamicInputYLocked)
        {
            const bool handled = m_editer->finishActivePolyline(m_drawState, false);

            if (handled && m_viewer != nullptr)
            {
                resetPointDynamicInputSession(currentPointInputStageKey());
                m_viewer->appendCommandMessage(QStringLiteral("已创建多段线图元"));
                m_viewer->refreshCommandPrompt();
            }

            return true;
        }

        return submitPointDynamicFieldInput();
    }

    if (!m_drawState.dynamicInputBuffer.isEmpty())
    {
        return submitDynamicInputBuffer();
    }

    if ((m_drawState.drawType == DrawType::Polyline || m_drawState.drawType == DrawType::LWPolyline)
        && m_editer != nullptr)
    {
        const bool handled = m_editer->finishActivePolyline(m_drawState, false);

        if (handled && m_viewer != nullptr)
        {
            resetPointDynamicInputSession(currentPointInputStageKey());
            m_viewer->appendCommandMessage(QStringLiteral("已创建多段线图元"));
            m_viewer->refreshCommandPrompt();
        }

        return true;
    }

    if (isAwaitingPointInput())
    {
        if (!commitCommandPoint(m_drawState.currentPos) && m_viewer != nullptr)
        {
            m_viewer->refreshCommandPrompt();
        }

        return true;
    }

    return true;
}

QString CadController::appendDynamicInputPromptState(const QString& basePrompt) const
{
    QString prompt = basePrompt;

    if (isParameterInputCommandActive() && isAwaitingParameterFieldInput())
    {
        QString labelText;
        QString valueText;

        switch (m_parameterInputSession.command)
        {
        case ParameterInputCommand::Polygon:
            if (m_parameterInputSession.stageIndex == 0)
            {
                labelText = QStringLiteral("边数");
                valueText = QString::number(m_parameterInputSession.intValue1);
            }
            else
            {
                labelText = QStringLiteral("构造方式");
                valueText = m_parameterInputSession.boolValue ? QStringLiteral("外切于圆") : QStringLiteral("内切于圆");
            }
            break;
        case ParameterInputCommand::Copy:
            labelText = QStringLiteral("位移");
            valueText = QStringLiteral("%1, %2")
                .arg(formatDynamicInputValue(m_parameterInputSession.doubleValue1))
                .arg(formatDynamicInputValue(m_parameterInputSession.doubleValue2));
            break;
        case ParameterInputCommand::Rotate:
            labelText = QStringLiteral("角度");
            valueText = formatDynamicInputValue(m_parameterInputSession.doubleValue1);
            break;
        case ParameterInputCommand::Scale:
            labelText = QStringLiteral("倍率");
            valueText = formatDynamicInputValue(m_parameterInputSession.doubleValue1);
            break;
        case ParameterInputCommand::RectangularArray:
            switch (m_parameterInputSession.stageIndex)
            {
            case 0:
                labelText = QStringLiteral("行数");
                valueText = QString::number(m_parameterInputSession.intValue1);
                break;
            case 1:
                labelText = QStringLiteral("列数");
                valueText = QString::number(m_parameterInputSession.intValue2);
                break;
            case 2:
                labelText = QStringLiteral("行间距");
                valueText = formatDynamicInputValue(m_parameterInputSession.doubleValue1);
                break;
            default:
                labelText = QStringLiteral("列间距");
                valueText = formatDynamicInputValue(m_parameterInputSession.doubleValue2);
                break;
            }
            break;
        case ParameterInputCommand::CircularArray:
            switch (m_parameterInputSession.stageIndex)
            {
            case 0:
                labelText = QStringLiteral("项目总数");
                valueText = QString::number(m_parameterInputSession.intValue1);
                break;
            case 1:
                labelText = QStringLiteral("填充角度");
                valueText = formatDynamicInputValue(m_parameterInputSession.doubleValue1);
                break;
            default:
                labelText = QStringLiteral("旋转方式");
                valueText = m_parameterInputSession.boolValue ? QStringLiteral("旋转副本方向") : QStringLiteral("保持原方向");
                break;
            }
            break;
        case ParameterInputCommand::Mirror:
            labelText = QStringLiteral("删除原图元");
            valueText = m_parameterInputSession.boolValue ? QStringLiteral("是") : QStringLiteral("否");
            break;
        case ParameterInputCommand::Offset:
            labelText = QStringLiteral("距离");
            valueText = formatDynamicInputValue(m_parameterInputSession.doubleValue1);
            break;
        case ParameterInputCommand::Trim:
        case ParameterInputCommand::Extend:
            labelText = QStringLiteral("端点");
            valueText = m_parameterInputSession.boolValue ? QStringLiteral("起点端") : QStringLiteral("终点端");
            break;
        case ParameterInputCommand::Fillet:
            labelText = QStringLiteral("半径");
            valueText = formatDynamicInputValue(m_parameterInputSession.doubleValue1);
            break;
        case ParameterInputCommand::Chamfer:
            labelText = m_parameterInputSession.stageIndex == 0 ? QStringLiteral("距离1") : QStringLiteral("距离2");
            valueText = formatDynamicInputValue(m_parameterInputSession.stageIndex == 0 ? m_parameterInputSession.doubleValue1 : m_parameterInputSession.doubleValue2);
            break;
        default:
            break;
        }

        if (!m_parameterInputSession.fieldBuffer.isEmpty())
        {
            valueText = m_parameterInputSession.fieldBuffer;
        }

        prompt += QStringLiteral(" | %1=%2").arg(labelText, valueText);
    }
    else if (m_drawState.hasActiveCommand() && isAwaitingPointInput())
    {
        if (m_drawState.dynamicInputExpressionMode)
        {
            if (!m_drawState.dynamicInputBuffer.isEmpty())
            {
                prompt += QStringLiteral(" | 表达式: %1").arg(m_drawState.dynamicInputBuffer);
            }
        }
        else
        {
            const QVector3D previewPoint = applyPointDynamicFieldOverride(m_drawState.currentPos, true);
            QString xText = formatDynamicInputValue(previewPoint.x());
            QString yText = formatDynamicInputValue(previewPoint.y());

            if (!m_drawState.dynamicInputFieldBuffer.isEmpty())
            {
                if (m_drawState.dynamicInputActiveFieldIndex == 0)
                {
                    xText = m_drawState.dynamicInputFieldBuffer;
                }
                else
                {
                    yText = m_drawState.dynamicInputFieldBuffer;
                }
            }

            prompt += QStringLiteral(" | X%1=%2 | Y%3=%4 | Tab切换")
                .arg(m_drawState.dynamicInputXLocked ? QStringLiteral("[锁]") : QString())
                .arg(xText)
                .arg(m_drawState.dynamicInputYLocked ? QStringLiteral("[锁]") : QString())
                .arg(yText);
        }
    }
    else if (isDynamicCommandModeActive())
    {
        const QVector<int> matchIndices = collectDynamicCommandMatchIndices();

        if (!matchIndices.isEmpty())
        {
            const int selectedOrdinal = std::clamp(m_drawState.dynamicCommandActiveIndex, 0, static_cast<int>(matchIndices.size()) - 1);
            const QVector<DynamicCommandDefinition>& definitions = dynamicCommandDefinitions();
            const QString selectedDisplay = definitions.at(matchIndices.at(selectedOrdinal)).displayName;
            prompt += QStringLiteral(" | 命令: %1 | 候选: %2").arg(m_drawState.dynamicCommandBuffer, selectedDisplay);
        }
        else
        {
            prompt += QStringLiteral(" | 命令: %1 | 无匹配").arg(m_drawState.dynamicCommandBuffer);
        }
    }

    if (m_drawState.orthoEnabled)
    {
        prompt += QStringLiteral(" | [正交]");
    }

    if (m_drawState.polarTrackingEnabled)
    {
        prompt += QStringLiteral(" | [极轴15°]");
    }

    return prompt;
}
