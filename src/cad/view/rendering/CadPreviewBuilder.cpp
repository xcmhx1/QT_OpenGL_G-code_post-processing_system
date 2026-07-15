// CadPreviewBuilder 实现文件
// 实现 CadPreviewBuilder 模块，对应头文件中声明的主要行为和协作流程。
// 预览构建模块，负责根据当前命令状态生成 transient 预览图元。

#include "platform/pch.h"

#include "cad/view/rendering/CadPreviewBuilder.h"
#include "compatibility/legacy/SplineEntityClone.h"

// CAD 模块内部依赖
#include "cad/document/CadDocument.h"
#include "cad/items/CadItem.h"
#include "cad/view/CadViewerUtils.h"
#include "cad/editing/DrawStateMachine.h"

// 标准库
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

// 匿名命名空间，存放局部常量和辅助函数
namespace
{
    // 圆周率常量
    constexpr float kPi = 3.14159265358979323846f;
    // 浮点数比较的微小阈值
    constexpr float kBasisEpsilon = 1.0e-8f;
    // 预览曲线默认分段数
    constexpr int kPreviewCurveSegments = 48;
    // 预览圆弧基础分段数
    constexpr int kPreviewBulgeBaseSegments = 64;
    // 几何精度阈值
    constexpr double kGeometryEpsilon = 1.0e-9;
    // 两倍圆周率
    constexpr double kTwoPi = 6.28318530717958647692;
    // 椭圆最小短长轴比
    constexpr double kMinEllipseRatio = 1.0e-4;
    constexpr float kXlinePreviewHalfLength = 65536.0f;

    // 夹点编辑预览颜色
    const QVector3D kGripPreviewColor(0.98f, 0.67f, 0.12f);
    const QVector3D kGripGuideColor(0.35f, 0.90f, 1.0f);
    const QVector3D kGripTargetColor(1.0f, 0.92f, 0.25f);

    // 将当前鼠标点投影到已知半径的圆上，便于圆/圆弧预览保持半径稳定
    // @param center 圆心坐标
    // @param radiusPoint 半径点坐标
    // @param currentPoint 当前鼠标位置
    // @return 投影到圆上的点坐标
    QVector3D projectPointToRadius(const QVector3D& center, const QVector3D& radiusPoint, const QVector3D& currentPoint)
    {
        // 计算半径
        const float radius = (radiusPoint - center).length();

        if (radius <= 1.0e-6f)
        {
            return radiusPoint;
        }

        QVector3D direction = currentPoint - center;

        if (direction.lengthSquared() <= 1.0e-10f)
        {
            direction = radiusPoint - center;
        }

        if (direction.lengthSquared() <= 1.0e-10f)
        {
            return radiusPoint;
        }

        direction.normalize();
        return center + direction * radius;
    }

    QVector3D rotatePlanarPreviewPoint(const QVector3D& point, const QVector3D& basePoint, double radians)
    {
        const double cosineValue = std::cos(radians);
        const double sineValue = std::sin(radians);
        const double dx = static_cast<double>(point.x() - basePoint.x());
        const double dy = static_cast<double>(point.y() - basePoint.y());

        return QVector3D
        (
            static_cast<float>(basePoint.x() + dx * cosineValue - dy * sineValue),
            static_cast<float>(basePoint.y() + dx * sineValue + dy * cosineValue),
            point.z()
        );
    }

    QVector3D mirrorPlanarPreviewPoint(const QVector3D& point, const QVector3D& lineStart, const QVector3D& lineEnd)
    {
        const QVector3D axis = lineEnd - lineStart;
        const double lengthSquared = static_cast<double>(axis.lengthSquared());

        if (lengthSquared <= kGeometryEpsilon)
        {
            return point;
        }

        const QVector3D fromStart = point - lineStart;
        const double t = QVector3D::dotProduct(fromStart, axis) / lengthSquared;
        const QVector3D projected = lineStart + axis * static_cast<float>(t);
        return projected * 2.0f - point;
    }

    QVector3D previewGeometryCenter(const CadItem* item)
    {
        if (item == nullptr || item->m_geometry.vertices.isEmpty())
        {
            return QVector3D();
        }

        QVector3D center;

        for (const QVector3D& vertex : item->m_geometry.vertices)
        {
            center += CadViewerUtils::flattenedToGroundPlane(vertex);
        }

        return center / static_cast<float>(item->m_geometry.vertices.size());
    }

    QVector3D offsetPreviewNormal(const QVector<QVector3D>& vertices)
    {
        if (vertices.size() < 2)
        {
            return QVector3D();
        }

        QVector3D direction = CadViewerUtils::flattenedToGroundPlane(vertices.back()) - CadViewerUtils::flattenedToGroundPlane(vertices.front());

        if (direction.lengthSquared() <= kBasisEpsilon)
        {
            return QVector3D();
        }

        direction.normalize();
        return QVector3D(-direction.y(), direction.x(), 0.0f);
    }

    QVector<QVector3D> buildOffsetPreviewVertices(const CadItem* item, double distance)
    {
        QVector<QVector3D> result;

        if (item == nullptr || item->m_geometry.vertices.isEmpty())
        {
            return result;
        }

        result.reserve(item->m_geometry.vertices.size());

        if ((item->m_type == DRW::ETYPE::CIRCLE || item->m_type == DRW::ETYPE::ARC)
            && item->m_nativeEntity != nullptr)
        {
            const DRW_Circle* circle = static_cast<const DRW_Circle*>(item->m_nativeEntity);
            const QVector3D center(circle->basePoint.x, circle->basePoint.y, circle->basePoint.z);
            const double targetRadius = std::max(circle->radious + distance, 0.001);
            const double factor = circle->radious > kGeometryEpsilon ? targetRadius / circle->radious : 1.0;

            for (const QVector3D& vertex : item->m_geometry.vertices)
            {
                const QVector3D planarVertex = CadViewerUtils::flattenedToGroundPlane(vertex);
                result.append(center + (planarVertex - center) * static_cast<float>(factor));
            }

            return result;
        }

        const QVector3D normal = offsetPreviewNormal(item->m_geometry.vertices);
        const QVector3D delta = normal * static_cast<float>(distance);

        for (const QVector3D& vertex : item->m_geometry.vertices)
        {
            result.append(CadViewerUtils::flattenedToGroundPlane(vertex) + delta);
        }

        return result;
    }

    void appendDashedLinePreview(QVector<QVector3D>& vertices, const QVector3D& startPoint, const QVector3D& endPoint)
    {
        constexpr int kDashCount = 16;
        constexpr float kDashRatio = 0.55f;

        const QVector3D delta = endPoint - startPoint;

        if (delta.lengthSquared() <= 1.0e-10f)
        {
            return;
        }

        vertices.reserve(vertices.size() + kDashCount * 2);

        for (int index = 0; index < kDashCount; ++index)
        {
            const float segmentStart = static_cast<float>(index) / static_cast<float>(kDashCount);
            const float segmentEnd = (static_cast<float>(index) + kDashRatio) / static_cast<float>(kDashCount);
            vertices.append(startPoint + delta * segmentStart);
            vertices.append(startPoint + delta * std::min(segmentEnd, 1.0f));
        }
    }

    // 将圆预览离散为闭合折线
    // @param vertices 输出顶点列表
    // @param center 圆心坐标
    // @param radius 圆半径
    // @param segments 分段数，默认为kPreviewCurveSegments
    void appendCirclePreview(QVector<QVector3D>& vertices, const QVector3D& center, float radius, int segments = kPreviewCurveSegments)
    {
        // 半径过小则不生成预览
        if (radius <= 1.0e-6f)
        {
            return;
        }

        vertices.reserve(segments + 1);

        // 在圆周上均匀采样
        for (int i = 0; i <= segments; ++i)
        {
            const float angle = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(segments);
            vertices.append
            (
                QVector3D
                (
                    center.x() + std::cos(angle) * radius,
                    center.y() + std::sin(angle) * radius,
                    center.z()
                )
            );
        }
    }

    // 生成圆弧预览
    // @param vertices 输出顶点列表
    // @param center 圆心坐标
    // @param startPoint 起点坐标
    // @param endPoint 终点坐标
    // @param segments 分段数，默认为kPreviewCurveSegments
    void appendArcPreview
    (
        QVector<QVector3D>& vertices,
        const QVector3D& center,
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        bool useComplementArc = false,
        int segments = kPreviewCurveSegments
    )
    {
        const QVector3D previewStartPoint = useComplementArc ? endPoint : startPoint;
        const QVector3D previewEndPoint = useComplementArc ? startPoint : endPoint;
        // 计算圆弧半径
        const float radius = (previewStartPoint - center).length();

        if (radius <= 1.0e-6f)
        {
            return;
        }

        // 计算起始角度和终止角度
        float startAngle = std::atan2(previewStartPoint.y() - center.y(), previewStartPoint.x() - center.x());
        float endAngle = std::atan2(previewEndPoint.y() - center.y(), previewEndPoint.x() - center.x());

        // 展开到正向区间，避免结束角小于开始角时预览反向跳变
        while (endAngle <= startAngle)
        {
            endAngle += 2.0f * kPi;
        }

        vertices.reserve(segments + 1);

        // 在角度区间内均匀采样
        for (int i = 0; i <= segments; ++i)
        {
            const float t = startAngle + (endAngle - startAngle) * static_cast<float>(i) / static_cast<float>(segments);
            vertices.append
            (
                QVector3D
                (
                    center.x() + std::cos(t) * radius,
                    center.y() + std::sin(t) * radius,
                    center.z()
                )
            );
        }
    }

    // 生成椭圆预览
    // @param vertices 输出顶点列表
    // @param center 椭圆中心
    // @param majorAxisPoint 长轴端点
    // @param ratioPoint 用于确定短轴长度的点
    // @param segments 分段数，默认为kPreviewCurveSegments
    void appendEllipsePreview
    (
        QVector<QVector3D>& vertices,
        const QVector3D& center,
        const QVector3D& majorAxisPoint,
        const QVector3D& ratioPoint,
        int segments = kPreviewCurveSegments
    )
    {
        // 计算长轴向量和长度
        const QVector3D majorAxis = majorAxisPoint - center;
        const float majorLength = majorAxis.length();

        if (majorLength <= 1.0e-6f)
        {
            return;
        }

        // ratioPoint 用于确定短轴长度，先把它在长轴方向上的分量扣掉
        const QVector3D toRatioPoint = ratioPoint - center;
        const float projectedLength = QVector3D::dotProduct(toRatioPoint, majorAxis) / majorLength;
        const float minorSquared = std::max(0.0f, toRatioPoint.lengthSquared() - projectedLength * projectedLength);
        const float minorLength = std::sqrt(minorSquared);

        if (minorLength <= 1.0e-6f)
        {
            return;
        }

        // 这里默认在当前二维绘图平面里构造与长轴正交的短轴方向
        QVector3D majorDirection = majorAxis;
        majorDirection.normalize();
        const QVector3D minorDirection(-majorDirection.y(), majorDirection.x(), 0.0f);

        vertices.reserve(segments + 1);

        // 使用椭圆参数方程生成顶点
        for (int i = 0; i <= segments; ++i)
        {
            const float angle = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(segments);
            vertices.append
            (
                center
                + majorDirection * (std::cos(angle) * majorLength)
                + minorDirection * (std::sin(angle) * minorLength)
            );
        }
    }

    void appendRectanglePreview(QVector<QVector3D>& vertices, const QVector3D& firstCorner, const QVector3D& secondCorner)
    {
        const QVector3D planarFirstCorner = CadViewerUtils::flattenedToGroundPlane(firstCorner);
        const QVector3D planarSecondCorner = CadViewerUtils::flattenedToGroundPlane(secondCorner);

        if (std::abs(planarSecondCorner.x() - planarFirstCorner.x()) <= 1.0e-10f
            || std::abs(planarSecondCorner.y() - planarFirstCorner.y()) <= 1.0e-10f)
        {
            return;
        }

        vertices.reserve(vertices.size() + 5);
        vertices.append(planarFirstCorner);
        vertices.append(QVector3D(planarSecondCorner.x(), planarFirstCorner.y(), 0.0f));
        vertices.append(planarSecondCorner);
        vertices.append(QVector3D(planarFirstCorner.x(), planarSecondCorner.y(), 0.0f));
        vertices.append(planarFirstCorner);
    }

    void appendPolygonPreview
    (
        QVector<QVector3D>& vertices,
        const QVector3D& center,
        const QVector3D& radiusPoint,
        int sideCount,
        bool circumscribedAboutCircle
    )
    {
        if (sideCount < 3 || sideCount > 1024)
        {
            return;
        }

        const QVector3D planarCenter = CadViewerUtils::flattenedToGroundPlane(center);
        const QVector3D planarRadiusPoint = CadViewerUtils::flattenedToGroundPlane(radiusPoint);
        const QVector3D radiusVector = planarRadiusPoint - planarCenter;
        const double referenceRadius = radiusVector.length();

        if (referenceRadius <= kGeometryEpsilon)
        {
            return;
        }

        const double stepAngle = kTwoPi / static_cast<double>(sideCount);
        const double baseAngle = std::atan2(radiusVector.y(), radiusVector.x());
        const double startAngle = circumscribedAboutCircle ? (baseAngle + stepAngle * 0.5) : baseAngle;
        const double polygonRadius = circumscribedAboutCircle
            ? (referenceRadius / std::cos(stepAngle * 0.5))
            : referenceRadius;

        if (!std::isfinite(polygonRadius) || polygonRadius <= kGeometryEpsilon)
        {
            return;
        }

        vertices.reserve(vertices.size() + sideCount + 1);

        for (int index = 0; index <= sideCount; ++index)
        {
            const double angle = startAngle + stepAngle * static_cast<double>(index % sideCount);
            vertices.append
            (
                QVector3D
                (
                    static_cast<float>(planarCenter.x() + polygonRadius * std::cos(angle)),
                    static_cast<float>(planarCenter.y() + polygonRadius * std::sin(angle)),
                    0.0f
                )
            );
        }
    }

    // 安全归一化向量，避免对零向量归一化产生NaN
    // @param vector 输入向量
    // @return 归一化后的向量，如果输入向量接近零则返回零向量
    QVector3D normalizedOrZero(const QVector3D& vector)
    {
        if (vector.lengthSquared() <= kBasisEpsilon)
        {
            return QVector3D();
        }

        QVector3D normalized = vector;
        normalized.normalize();
        return normalized;
    }

    // 生成多段线圆弧段预览
    // @param vertices 输出顶点列表
    // @param startPoint 起点
    // @param endPoint 终点
    // @param bulge 圆弧凸度值
    void appendBulgePreview(QVector<QVector3D>& vertices, const QVector3D& startPoint, const QVector3D& endPoint, double bulge)
    {
        const double dx = endPoint.x() - startPoint.x();
        const double dy = endPoint.y() - startPoint.y();
        const double chordLength = std::sqrt(dx * dx + dy * dy);

        // bulge 为 0 或弦长退化时，直接追加终点即可
        if (chordLength <= 1.0e-10 || std::abs(bulge) < 1.0e-8)
        {
            vertices.append(endPoint);
            return;
        }

        // 由 bulge 恢复圆心、半径与扫角，再按弧长展开中间采样点
        const double midpointX = (startPoint.x() + endPoint.x()) * 0.5;
        const double midpointY = (startPoint.y() + endPoint.y()) * 0.5;
        const double centerOffset = chordLength * (1.0 / bulge - bulge) * 0.25;
        const double centerX = midpointX - centerOffset * (dy / chordLength);
        const double centerY = midpointY + centerOffset * (dx / chordLength);
        const double radius = std::hypot(startPoint.x() - centerX, startPoint.y() - centerY);
        const double startAngle = std::atan2(startPoint.y() - centerY, startPoint.x() - centerX);
        const double sweepAngle = 4.0 * std::atan(bulge);
        const int segments = std::max(4, static_cast<int>(std::ceil(std::abs(sweepAngle) / (2.0 * kPi) * kPreviewBulgeBaseSegments)));

        // 在圆弧上均匀采样
        for (int i = 1; i <= segments; ++i)
        {
            const double factor = static_cast<double>(i) / static_cast<double>(segments);
            const double angle = startAngle + sweepAngle * factor;

            vertices.append
            (
                QVector3D
                (
                    static_cast<float>(centerX + radius * std::cos(angle)),
                    static_cast<float>(centerY + radius * std::sin(angle)),
                    0.0f
                )
            );
        }
    }

    // 从起点、终点和bulge计算圆弧圆心
    // @param startPoint 起点
    // @param endPoint 终点
    // @param bulge 圆弧凸度值
    // @param valid 可选输出参数，指示计算是否有效
    // @return 圆心坐标，如果无效则返回零向量
    QVector3D bulgeArcCenter(const QVector3D& startPoint, const QVector3D& endPoint, double bulge, bool* valid = nullptr)
    {
        const QVector3D chord = endPoint - startPoint;
        const double chordLength = chord.length();

        if (valid != nullptr)
        {
            *valid = false;
        }

        if (chordLength <= 1.0e-10 || std::abs(bulge) < 1.0e-8)
        {
            return QVector3D();
        }

        const QVector3D midpoint = (startPoint + endPoint) * 0.5f;
        const double centerOffset = chordLength * (1.0 / bulge - bulge) * 0.25;
        const QVector3D leftNormal
        (
            static_cast<float>(-chord.y() / chordLength),
            static_cast<float>(chord.x() / chordLength),
            0.0f
        );

        if (valid != nullptr)
        {
            *valid = true;
        }

        return midpoint + leftNormal * static_cast<float>(centerOffset);
    }

    // 计算多段线末段的切向，用于圆弧续接预览
    // @param points 多段线顶点列表
    // @param bulges 多段线凸度列表
    // @return 末段切向向量
    QVector3D polylineEndTangent(const QVector<QVector3D>& points, const QVector<double>& bulges)
    {
        if (points.size() < 2)
        {
            return QVector3D();
        }

        const QVector3D startPoint = CadViewerUtils::flattenedToGroundPlane(points[points.size() - 2]);
        const QVector3D endPoint = CadViewerUtils::flattenedToGroundPlane(points.back());
        const double bulge = bulges.size() >= points.size() - 1 ? bulges[points.size() - 2] : 0.0;

        // 直线段的切向就是最后一段的方向
        if (std::abs(bulge) < 1.0e-8)
        {
            return normalizedOrZero(endPoint - startPoint);
        }

        bool valid = false;
        const QVector3D center = bulgeArcCenter(startPoint, endPoint, bulge, &valid);

        if (!valid)
        {
            return normalizedOrZero(endPoint - startPoint);
        }

        // 圆弧段的末端切向与半径向量垂直，并由 bulge 正负决定朝向
        const QVector3D radiusVector = endPoint - center;
        const QVector3D tangent = bulge > 0.0
            ? QVector3D(-radiusVector.y(), radiusVector.x(), 0.0f)
            : QVector3D(radiusVector.y(), -radiusVector.x(), 0.0f);

        return normalizedOrZero(tangent);
    }

    // 根据起点切向和终点反推 bulge，供多段线圆弧预览实时续接
    // @param startPoint 起点
    // @param tangentDirection 起点切向
    // @param endPoint 终点
    // @return 计算得到的凸度值
    double bulgeFromTangent(const QVector3D& startPoint, const QVector3D& tangentDirection, const QVector3D& endPoint)
    {
        const QVector3D planarTangent = normalizedOrZero(QVector3D(tangentDirection.x(), tangentDirection.y(), 0.0f));
        const QVector3D chordVector = CadViewerUtils::flattenedToGroundPlane(endPoint) - CadViewerUtils::flattenedToGroundPlane(startPoint);

        if (planarTangent.lengthSquared() <= 1.0e-10 || chordVector.lengthSquared() <= 1.0e-10)
        {
            return 0.0;
        }

        const double dotValue = QVector3D::dotProduct(planarTangent, chordVector);
        const double crossValue = planarTangent.x() * chordVector.y() - planarTangent.y() * chordVector.x();
        const double alpha = std::atan2(crossValue, dotValue);

        // 当切向与弦方向接近 180 度时会趋向无穷大，这里显式处理
        if (std::abs(std::abs(alpha) - kPi) <= 1.0e-6)
        {
            return std::numeric_limits<double>::infinity();
        }

        return std::tan(alpha * 0.5);
    }

    QVector3D flattenToDrawingPlane(const QVector3D& point)
    {
        return QVector3D(point.x(), point.y(), 0.0f);
    }

    double normalizeAnglePositive(double angle)
    {
        double normalized = std::fmod(angle, kTwoPi);

        if (normalized < 0.0)
        {
            normalized += kTwoPi;
        }

        return normalized;
    }

    QVector3D resolveEntityNormal(const DRW_Coord& extPoint)
    {
        QVector3D normal(extPoint.x, extPoint.y, extPoint.z);

        if (normal.lengthSquared() <= kGeometryEpsilon)
        {
            return QVector3D(0.0f, 0.0f, 1.0f);
        }

        normal.normalize();
        return normal;
    }

    void buildPlaneBasis(const QVector3D& normal, QVector3D& axisX, QVector3D& axisY)
    {
        if (std::abs(normal.x()) <= 1.0e-6f && std::abs(normal.y()) <= 1.0e-6f)
        {
            axisX = QVector3D(1.0f, 0.0f, 0.0f);
            axisY = QVector3D::crossProduct(normal, axisX);

            if (axisY.lengthSquared() <= kGeometryEpsilon)
            {
                axisY = QVector3D(0.0f, 1.0f, 0.0f);
            }
            else
            {
                axisY.normalize();
            }

            return;
        }

        const QVector3D helper = std::abs(normal.z()) < 0.999f
            ? QVector3D(0.0f, 0.0f, 1.0f)
            : QVector3D(0.0f, 1.0f, 0.0f);

        axisX = QVector3D::crossProduct(helper, normal);

        if (axisX.lengthSquared() <= kGeometryEpsilon)
        {
            axisX = QVector3D(1.0f, 0.0f, 0.0f);
        }
        else
        {
            axisX.normalize();
        }

        axisY = QVector3D::crossProduct(normal, axisX);

        if (axisY.lengthSquared() <= kGeometryEpsilon)
        {
            axisY = QVector3D(0.0f, 1.0f, 0.0f);
        }
        else
        {
            axisY.normalize();
        }
    }

    double angleFromPointOnCircle(const DRW_Circle* circle, const QVector3D& point, bool* valid = nullptr)
    {
        if (valid != nullptr)
        {
            *valid = false;
        }

        if (circle == nullptr)
        {
            return 0.0;
        }

        const QVector3D center(circle->basePoint.x, circle->basePoint.y, circle->basePoint.z);
        const QVector3D local = flattenToDrawingPlane(point) - center;

        if (local.lengthSquared() <= kGeometryEpsilon)
        {
            return 0.0;
        }

        const QVector3D normal = resolveEntityNormal(circle->extPoint);
        QVector3D axisX;
        QVector3D axisY;
        buildPlaneBasis(normal, axisX, axisY);

        const double x = QVector3D::dotProduct(local, axisX);
        const double y = QVector3D::dotProduct(local, axisY);

        if (std::abs(x) <= kGeometryEpsilon && std::abs(y) <= kGeometryEpsilon)
        {
            return 0.0;
        }

        if (valid != nullptr)
        {
            *valid = true;
        }

        return std::atan2(y, x);
    }

    bool tryBuildEllipseAxes(const DRW_Ellipse* ellipse, QVector3D& majorAxis, QVector3D& minorAxis)
    {
        if (ellipse == nullptr)
        {
            return false;
        }

        majorAxis = QVector3D(ellipse->secPoint.x, ellipse->secPoint.y, ellipse->secPoint.z);
        const double majorLength = majorAxis.length();

        if (majorLength <= kGeometryEpsilon || ellipse->ratio <= kMinEllipseRatio)
        {
            return false;
        }

        const QVector3D normal = resolveEntityNormal(ellipse->extPoint);
        minorAxis = QVector3D::crossProduct(normal, majorAxis);

        if (minorAxis.lengthSquared() <= kGeometryEpsilon)
        {
            return false;
        }

        minorAxis.normalize();
        minorAxis *= static_cast<float>(majorLength * ellipse->ratio);
        return true;
    }

    bool ellipseParameterFromPoint(const DRW_Ellipse* ellipse, const QVector3D& worldPoint, double& parameter)
    {
        QVector3D majorAxis;
        QVector3D minorAxis;

        if (ellipse == nullptr || !tryBuildEllipseAxes(ellipse, majorAxis, minorAxis))
        {
            return false;
        }

        const double majorLength = majorAxis.length();
        const double minorLength = minorAxis.length();

        if (majorLength <= kGeometryEpsilon || minorLength <= kGeometryEpsilon)
        {
            return false;
        }

        QVector3D majorUnit = majorAxis;
        QVector3D minorUnit = minorAxis;
        majorUnit.normalize();
        minorUnit.normalize();

        const QVector3D center(ellipse->basePoint.x, ellipse->basePoint.y, ellipse->basePoint.z);
        const QVector3D local = flattenToDrawingPlane(worldPoint) - center;
        const double x = QVector3D::dotProduct(local, majorUnit);
        const double y = QVector3D::dotProduct(local, minorUnit);
        const double cosValue = x / majorLength;
        const double sinValue = y / minorLength;
        parameter = std::atan2(sinValue, cosValue);
        return true;
    }

    std::unique_ptr<DRW_Entity> cloneEntityForPreview(const DRW_Entity* entity)
    {
        if (entity == nullptr)
        {
            return nullptr;
        }

        switch (entity->eType)
        {
        case DRW::ETYPE::POINT:
            return std::make_unique<DRW_Point>(*static_cast<const DRW_Point*>(entity));
        case DRW::ETYPE::LINE:
            return std::make_unique<DRW_Line>(*static_cast<const DRW_Line*>(entity));
        case DRW::ETYPE::XLINE:
            return std::make_unique<DRW_Xline>(*static_cast<const DRW_Xline*>(entity));
        case DRW::ETYPE::CIRCLE:
            return std::make_unique<DRW_Circle>(*static_cast<const DRW_Circle*>(entity));
        case DRW::ETYPE::ARC:
            return std::make_unique<DRW_Arc>(*static_cast<const DRW_Arc*>(entity));
        case DRW::ETYPE::ELLIPSE:
            return std::make_unique<DRW_Ellipse>(*static_cast<const DRW_Ellipse*>(entity));
        case DRW::ETYPE::LWPOLYLINE:
            return std::make_unique<DRW_LWPolyline>(*static_cast<const DRW_LWPolyline*>(entity));
        case DRW::ETYPE::POLYLINE:
            return std::make_unique<DRW_Polyline>(*static_cast<const DRW_Polyline*>(entity));
        case DRW::ETYPE::SPLINE:
            return cloneSplineEntity(static_cast<const DRW_Spline*>(entity));
        default:
            return nullptr;
        }
    }

    bool applyEditableControlPoint(DRW_Entity* entity, int pointIndex, const QVector3D& worldPos)
    {
        if (entity == nullptr || pointIndex < 0)
        {
            return false;
        }

        const QVector3D point = flattenToDrawingPlane(worldPos);

        switch (entity->eType)
        {
        case DRW::ETYPE::POINT:
        {
            if (pointIndex != 0)
            {
                return false;
            }

            DRW_Point* pointEntity = static_cast<DRW_Point*>(entity);
            pointEntity->basePoint.x = point.x();
            pointEntity->basePoint.y = point.y();
            pointEntity->basePoint.z = point.z();
            return true;
        }
        case DRW::ETYPE::LINE:
        {
            DRW_Line* line = static_cast<DRW_Line*>(entity);

            if (pointIndex == 0)
            {
                line->basePoint.x = point.x();
                line->basePoint.y = point.y();
                line->basePoint.z = point.z();
                return true;
            }

            if (pointIndex == 1)
            {
                line->secPoint.x = point.x();
                line->secPoint.y = point.y();
                line->secPoint.z = point.z();
                return true;
            }

            if (pointIndex == 2)
            {
                const QVector3D startPoint(line->basePoint.x, line->basePoint.y, line->basePoint.z);
                const QVector3D endPoint(line->secPoint.x, line->secPoint.y, line->secPoint.z);
                const QVector3D delta = point - (startPoint + endPoint) * 0.5f;
                line->basePoint.x += delta.x();
                line->basePoint.y += delta.y();
                line->basePoint.z += delta.z();
                line->secPoint.x += delta.x();
                line->secPoint.y += delta.y();
                line->secPoint.z += delta.z();
                return true;
            }

            return false;
        }
        case DRW::ETYPE::XLINE:
        {
            DRW_Xline* xline = static_cast<DRW_Xline*>(entity);
            const QVector3D basePoint(xline->basePoint.x, xline->basePoint.y, xline->basePoint.z);

            if (pointIndex == 0)
            {
                xline->basePoint.x = point.x();
                xline->basePoint.y = point.y();
                xline->basePoint.z = point.z();
                return true;
            }

            if (pointIndex == 1 || pointIndex == 2)
            {
                QVector3D direction = pointIndex == 1
                    ? QVector3D(point.x() - basePoint.x(), point.y() - basePoint.y(), 0.0f)
                    : QVector3D(basePoint.x() - point.x(), basePoint.y() - point.y(), 0.0f);

                if (direction.lengthSquared() <= kGeometryEpsilon)
                {
                    return false;
                }

                direction.normalize();
                xline->secPoint.x = direction.x();
                xline->secPoint.y = direction.y();
                xline->secPoint.z = direction.z();
                return true;
            }

            return false;
        }
        case DRW::ETYPE::POLYLINE:
        {
            DRW_Polyline* polyline = static_cast<DRW_Polyline*>(entity);

            if (pointIndex >= static_cast<int>(polyline->vertlist.size()))
            {
                return false;
            }

            const std::shared_ptr<DRW_Vertex>& vertex = polyline->vertlist[static_cast<size_t>(pointIndex)];

            if (vertex == nullptr)
            {
                return false;
            }

            vertex->basePoint.x = point.x();
            vertex->basePoint.y = point.y();
            vertex->basePoint.z = point.z();
            return true;
        }
        case DRW::ETYPE::LWPOLYLINE:
        {
            DRW_LWPolyline* polyline = static_cast<DRW_LWPolyline*>(entity);

            if (pointIndex >= static_cast<int>(polyline->vertlist.size()))
            {
                return false;
            }

            const std::shared_ptr<DRW_Vertex2D>& vertex = polyline->vertlist[static_cast<size_t>(pointIndex)];

            if (vertex == nullptr)
            {
                return false;
            }

            vertex->x = point.x();
            vertex->y = point.y();
            return true;
        }
        case DRW::ETYPE::CIRCLE:
        {
            DRW_Circle* circle = static_cast<DRW_Circle*>(entity);
            const QVector3D center(circle->basePoint.x, circle->basePoint.y, circle->basePoint.z);

            if (pointIndex == 0)
            {
                circle->basePoint.x = point.x();
                circle->basePoint.y = point.y();
                circle->basePoint.z = point.z();
                return true;
            }

            if (pointIndex >= 1 && pointIndex <= 4)
            {
                const double radius = (point - center).length();

                if (radius <= kGeometryEpsilon)
                {
                    return false;
                }

                circle->radious = radius;
                return true;
            }

            return false;
        }
        case DRW::ETYPE::ARC:
        {
            DRW_Arc* arc = static_cast<DRW_Arc*>(entity);
            DRW_Circle circleProxy;
            circleProxy.basePoint = arc->basePoint;
            circleProxy.extPoint = arc->extPoint;
            circleProxy.radious = arc->radious;

            if (pointIndex == 0)
            {
                arc->basePoint.x = point.x();
                arc->basePoint.y = point.y();
                arc->basePoint.z = point.z();
                return true;
            }

            if (pointIndex == 2)
            {
                const QVector3D center(arc->basePoint.x, arc->basePoint.y, arc->basePoint.z);
                const double radius = (point - center).length();

                if (radius <= kGeometryEpsilon)
                {
                    return false;
                }

                arc->radious = radius;
                return true;
            }

            bool validAngle = false;
            const double targetAngle = angleFromPointOnCircle(&circleProxy, point, &validAngle);

            if (!validAngle)
            {
                return false;
            }

            if (pointIndex == 1)
            {
                arc->staangle = normalizeAnglePositive(targetAngle);
                arc->endangle = normalizeAnglePositive(arc->endangle);

                while (arc->endangle <= arc->staangle)
                {
                    arc->endangle += kTwoPi;
                }

                return true;
            }

            if (pointIndex == 3)
            {
                arc->staangle = normalizeAnglePositive(arc->staangle);
                arc->endangle = normalizeAnglePositive(targetAngle);

                while (arc->endangle <= arc->staangle)
                {
                    arc->endangle += kTwoPi;
                }

                return true;
            }

            return false;
        }
        case DRW::ETYPE::ELLIPSE:
        {
            DRW_Ellipse* ellipse = static_cast<DRW_Ellipse*>(entity);
            QVector3D majorAxis;
            QVector3D minorAxis;

            if (!tryBuildEllipseAxes(ellipse, majorAxis, minorAxis))
            {
                return false;
            }

            const QVector3D center(ellipse->basePoint.x, ellipse->basePoint.y, ellipse->basePoint.z);
            const double majorLength = majorAxis.length();

            if (majorLength <= kGeometryEpsilon)
            {
                return false;
            }

            QVector3D majorUnit = majorAxis;
            QVector3D minorUnit = minorAxis;
            majorUnit.normalize();
            minorUnit.normalize();

            if (pointIndex == 0)
            {
                ellipse->basePoint.x = point.x();
                ellipse->basePoint.y = point.y();
                ellipse->basePoint.z = point.z();
                return true;
            }

            if (pointIndex == 1 || pointIndex == 2)
            {
                const QVector3D direction = pointIndex == 1 ? (point - center) : (center - point);

                if (direction.lengthSquared() <= kGeometryEpsilon)
                {
                    return false;
                }

                ellipse->secPoint.x = direction.x();
                ellipse->secPoint.y = direction.y();
                ellipse->secPoint.z = direction.z();
                return true;
            }

            if (pointIndex == 3 || pointIndex == 4)
            {
                const QVector3D local = point - center;
                const double minorLength = std::abs(QVector3D::dotProduct(local, minorUnit));

                if (minorLength <= kGeometryEpsilon)
                {
                    return false;
                }

                ellipse->ratio = std::clamp(minorLength / majorLength, kMinEllipseRatio, 1.0 - 1.0e-6);
                return true;
            }

            if (pointIndex == 5 || pointIndex == 6)
            {
                double parameter = 0.0;

                if (!ellipseParameterFromPoint(ellipse, point, parameter))
                {
                    return false;
                }

                if (pointIndex == 5)
                {
                    ellipse->staparam = normalizeAnglePositive(parameter);
                }
                else
                {
                    ellipse->endparam = normalizeAnglePositive(parameter);
                }

                return true;
            }

            return false;
        }
        default:
            return false;
        }
    }

    bool buildGripEditedEntityPreview
    (
        const CadItem* selectedItem,
        int gripPointIndex,
        const QVector3D& targetPoint,
        TransientPrimitive& preview
    )
    {
        if (selectedItem == nullptr || selectedItem->m_nativeEntity == nullptr || gripPointIndex < 0)
        {
            return false;
        }

        std::unique_ptr<DRW_Entity> previewEntity = cloneEntityForPreview(selectedItem->m_nativeEntity);

        if (previewEntity == nullptr || !applyEditableControlPoint(previewEntity.get(), gripPointIndex, targetPoint))
        {
            return false;
        }

        std::unique_ptr<CadItem> previewItem = CadDocument::createCadItemForEntity(previewEntity.get());

        if (previewItem == nullptr)
        {
            return false;
        }

        previewItem->buildGeometryDatay();
        preview.primitiveType = CadViewerUtils::primitiveTypeForEntity(previewItem.get());
        preview.color = kGripPreviewColor;
        preview.pointSize = preview.primitiveType == GL_POINTS ? 12.0f : 1.0f;
        preview.roundPoint = preview.primitiveType == GL_POINTS;
        preview.vertices = previewItem->m_geometry.vertices;
        return !preview.vertices.isEmpty();
    }
}

// CadPreviewBuilder 命名空间实现
namespace CadPreviewBuilder
{
    // 根据当前命令状态生成 transient 图元
    // 这些图元只参与预览渲染，不会写入文档模型
    // @param state 当前绘图状态机
    // @param selectedItem 当前选中的实体
    // @return 临时图元向量
    std::vector<TransientPrimitive> buildTransientPrimitives
    (
        const DrawStateMachine& state,
        CadItem* selectedItem,
        const QVector<CadItem*>& selectedItems
    )
    {
        std::vector<TransientPrimitive> primitives;

        // 没有活动命令时不生成任何预览图元
        if (!state.hasActiveCommand())
        {
            return primitives;
        }

        // 处理移动编辑命令的预览
        if (state.editType == EditType::Move)
        {
            // Move 命令预览由"整体平移后的实体"加"一条基点到目标点引导线"组成
            if (state.moveSubMode == MoveEditSubMode::AwaitTargetPoint
                && !state.commandPoints.isEmpty()
                && !selectedItems.isEmpty())
            {
                const QVector3D basePoint = CadViewerUtils::flattenedToGroundPlane(state.commandPoints.front());
                const QVector3D currentPoint = CadViewerUtils::flattenedToGroundPlane(state.currentPos);
                const QVector3D delta = currentPoint - basePoint;

                for (CadItem* item : selectedItems)
                {
                    if (item == nullptr || item->m_geometry.vertices.isEmpty())
                    {
                        continue;
                    }

                    // 创建移动后的实体预览
                    TransientPrimitive movedEntityPreview;
                    movedEntityPreview.primitiveType = CadViewerUtils::primitiveTypeForEntity(item);
                    movedEntityPreview.color = QVector3D(0.98f, 0.67f, 0.12f);  // 橙色
                    movedEntityPreview.pointSize = movedEntityPreview.primitiveType == GL_POINTS ? 12.0f : 1.0f;
                    movedEntityPreview.roundPoint = movedEntityPreview.primitiveType == GL_POINTS;

                    movedEntityPreview.vertices.reserve(item->m_geometry.vertices.size());

                    for (const QVector3D& vertex : item->m_geometry.vertices)
                    {
                        movedEntityPreview.vertices.append(vertex + delta);
                    }

                    if (!movedEntityPreview.vertices.isEmpty())
                    {
                        primitives.push_back(std::move(movedEntityPreview));
                    }
                }

                // 创建引导线预览
                TransientPrimitive guideLine;
                guideLine.primitiveType = GL_LINES;
                guideLine.color = QVector3D(0.35f, 0.90f, 1.0f);  // 青色
                guideLine.vertices = { basePoint, currentPoint };
                primitives.push_back(std::move(guideLine));
            }

            return primitives;
        }

        // 处理控制点编辑命令预览
        if (state.editType == EditType::GripEdit)
        {
            CadItem* gripTarget = selectedItem;

            if (gripTarget == nullptr && !selectedItems.isEmpty())
            {
                gripTarget = selectedItems.front();
            }

            if (gripTarget != nullptr
                && state.gripSubMode == GripEditSubMode::AwaitTargetPoint
                && state.gripPointIndex >= 0
                && !state.commandPoints.isEmpty())
            {
                const QVector3D basePoint = CadViewerUtils::flattenedToGroundPlane(state.commandPoints.front());
                const QVector3D targetPoint = CadViewerUtils::flattenedToGroundPlane(state.currentPos);

                TransientPrimitive entityPreview;

                if (buildGripEditedEntityPreview(gripTarget, state.gripPointIndex, targetPoint, entityPreview))
                {
                    primitives.push_back(std::move(entityPreview));
                }

                // 基点到目标点的引导线
                TransientPrimitive guideLine;
                guideLine.primitiveType = GL_LINES;
                guideLine.color = kGripGuideColor;
                guideLine.vertices = { basePoint, targetPoint };
                primitives.push_back(std::move(guideLine));

                // 当前目标点高亮
                TransientPrimitive targetPointMarker;
                targetPointMarker.primitiveType = GL_POINTS;
                targetPointMarker.color = kGripTargetColor;
                targetPointMarker.pointSize = 11.0f;
                targetPointMarker.roundPoint = true;
                targetPointMarker.vertices = { targetPoint };
                primitives.push_back(std::move(targetPointMarker));
            }

            return primitives;
        }

        if (state.copyPreviewActive && !selectedItems.isEmpty())
        {
            const QVector3D basePoint = CadViewerUtils::flattenedToGroundPlane(state.copyPreviewBasePoint);
            const QVector3D targetPoint = basePoint + CadViewerUtils::flattenedToGroundPlane(state.copyPreviewDelta);

            for (CadItem* item : selectedItems)
            {
                if (item == nullptr || item->m_geometry.vertices.isEmpty())
                {
                    continue;
                }

                TransientPrimitive copiedEntityPreview;
                copiedEntityPreview.primitiveType = CadViewerUtils::primitiveTypeForEntity(item);
                copiedEntityPreview.color = QVector3D(0.98f, 0.67f, 0.12f);
                copiedEntityPreview.pointSize = copiedEntityPreview.primitiveType == GL_POINTS ? 12.0f : 1.0f;
                copiedEntityPreview.roundPoint = copiedEntityPreview.primitiveType == GL_POINTS;
                copiedEntityPreview.vertices.reserve(item->m_geometry.vertices.size());

                for (const QVector3D& vertex : item->m_geometry.vertices)
                {
                    copiedEntityPreview.vertices.append(CadViewerUtils::flattenedToGroundPlane(vertex) + state.copyPreviewDelta);
                }

                primitives.push_back(std::move(copiedEntityPreview));
            }

            TransientPrimitive guideLine;
            guideLine.primitiveType = GL_LINES;
            guideLine.color = QVector3D(0.35f, 0.90f, 1.0f);
            guideLine.vertices = { basePoint, targetPoint };
            primitives.push_back(std::move(guideLine));

            return primitives;
        }

        if (state.mirrorPreviewActive && !selectedItems.isEmpty())
        {
            const QVector3D firstPoint = CadViewerUtils::flattenedToGroundPlane(state.mirrorPreviewFirstPoint);
            const QVector3D secondPoint = CadViewerUtils::flattenedToGroundPlane(state.mirrorPreviewSecondPoint);

            for (CadItem* item : selectedItems)
            {
                if (item == nullptr || item->m_geometry.vertices.isEmpty())
                {
                    continue;
                }

                TransientPrimitive mirroredEntityPreview;
                mirroredEntityPreview.primitiveType = CadViewerUtils::primitiveTypeForEntity(item);
                mirroredEntityPreview.color = QVector3D(0.98f, 0.67f, 0.12f);
                mirroredEntityPreview.pointSize = mirroredEntityPreview.primitiveType == GL_POINTS ? 12.0f : 1.0f;
                mirroredEntityPreview.roundPoint = mirroredEntityPreview.primitiveType == GL_POINTS;
                mirroredEntityPreview.vertices.reserve(item->m_geometry.vertices.size());

                for (const QVector3D& vertex : item->m_geometry.vertices)
                {
                    mirroredEntityPreview.vertices.append(mirrorPlanarPreviewPoint(CadViewerUtils::flattenedToGroundPlane(vertex), firstPoint, secondPoint));
                }

                primitives.push_back(std::move(mirroredEntityPreview));
            }

            TransientPrimitive mirrorAxis;
            mirrorAxis.primitiveType = GL_LINES;
            mirrorAxis.color = QVector3D(0.35f, 0.90f, 1.0f);
            mirrorAxis.vertices = { firstPoint, secondPoint };
            primitives.push_back(std::move(mirrorAxis));

            return primitives;
        }

        if (state.rectangularArrayPreviewActive && !selectedItems.isEmpty())
        {
            const int rows = std::max(state.rectangularArrayPreviewRows, 1);
            const int columns = std::max(state.rectangularArrayPreviewColumns, 1);

            for (int row = 0; row < rows; ++row)
            {
                for (int column = 0; column < columns; ++column)
                {
                    if (row == 0 && column == 0)
                    {
                        continue;
                    }

                    const QVector3D delta = state.rectangularArrayPreviewRowOffset * static_cast<float>(row)
                        + state.rectangularArrayPreviewColumnOffset * static_cast<float>(column);

                    for (CadItem* item : selectedItems)
                    {
                        if (item == nullptr || item->m_geometry.vertices.isEmpty())
                        {
                            continue;
                        }

                        TransientPrimitive arrayEntityPreview;
                        arrayEntityPreview.primitiveType = CadViewerUtils::primitiveTypeForEntity(item);
                        arrayEntityPreview.color = QVector3D(0.98f, 0.67f, 0.12f);
                        arrayEntityPreview.pointSize = arrayEntityPreview.primitiveType == GL_POINTS ? 12.0f : 1.0f;
                        arrayEntityPreview.roundPoint = arrayEntityPreview.primitiveType == GL_POINTS;
                        arrayEntityPreview.vertices.reserve(item->m_geometry.vertices.size());

                        for (const QVector3D& vertex : item->m_geometry.vertices)
                        {
                            arrayEntityPreview.vertices.append(CadViewerUtils::flattenedToGroundPlane(vertex) + delta);
                        }

                        primitives.push_back(std::move(arrayEntityPreview));
                    }
                }
            }

            return primitives;
        }

        if (state.circularArrayPreviewActive && !selectedItems.isEmpty())
        {
            constexpr double kRadiansPerDegree = 3.14159265358979323846 / 180.0;
            const QVector3D center = CadViewerUtils::flattenedToGroundPlane(state.circularArrayPreviewCenter);
            const int count = std::max(state.circularArrayPreviewCount, 1);
            const double divisor = std::abs(std::abs(state.circularArrayPreviewTotalAngleDegrees) - 360.0) <= 1.0e-6
                ? static_cast<double>(count)
                : static_cast<double>(std::max(count - 1, 1));
            const double stepDegrees = state.circularArrayPreviewTotalAngleDegrees / divisor;

            for (int index = 1; index < count; ++index)
            {
                const double radians = stepDegrees * static_cast<double>(index) * kRadiansPerDegree;

                for (CadItem* item : selectedItems)
                {
                    if (item == nullptr || item->m_geometry.vertices.isEmpty())
                    {
                        continue;
                    }

                    const QVector3D itemCenter = previewGeometryCenter(item);
                    const QVector3D rotatedCenter = rotatePlanarPreviewPoint(itemCenter, center, radians);
                    const QVector3D delta = rotatedCenter - itemCenter;
                    TransientPrimitive arrayEntityPreview;
                    arrayEntityPreview.primitiveType = CadViewerUtils::primitiveTypeForEntity(item);
                    arrayEntityPreview.color = QVector3D(0.98f, 0.67f, 0.12f);
                    arrayEntityPreview.pointSize = arrayEntityPreview.primitiveType == GL_POINTS ? 12.0f : 1.0f;
                    arrayEntityPreview.roundPoint = arrayEntityPreview.primitiveType == GL_POINTS;
                    arrayEntityPreview.vertices.reserve(item->m_geometry.vertices.size());

                    for (const QVector3D& vertex : item->m_geometry.vertices)
                    {
                        const QVector3D planarVertex = CadViewerUtils::flattenedToGroundPlane(vertex);
                        arrayEntityPreview.vertices.append
                        (
                            state.circularArrayPreviewRotateItems
                                ? rotatePlanarPreviewPoint(planarVertex, center, radians)
                                : planarVertex + delta
                        );
                    }

                    primitives.push_back(std::move(arrayEntityPreview));
                }
            }

            TransientPrimitive centerMarker;
            centerMarker.primitiveType = GL_POINTS;
            centerMarker.color = QVector3D(1.0f, 0.92f, 0.25f);
            centerMarker.pointSize = 11.0f;
            centerMarker.roundPoint = true;
            centerMarker.vertices = { center };
            primitives.push_back(std::move(centerMarker));

            return primitives;
        }

        if (state.offsetPreviewActive && !selectedItems.isEmpty())
        {
            for (CadItem* item : selectedItems)
            {
                const QVector<QVector3D> offsetVertices = buildOffsetPreviewVertices(item, state.offsetPreviewDistance);

                if (item == nullptr || offsetVertices.isEmpty())
                {
                    continue;
                }

                TransientPrimitive offsetEntityPreview;
                offsetEntityPreview.primitiveType = CadViewerUtils::primitiveTypeForEntity(item);
                offsetEntityPreview.color = QVector3D(0.98f, 0.67f, 0.12f);
                offsetEntityPreview.pointSize = offsetEntityPreview.primitiveType == GL_POINTS ? 12.0f : 1.0f;
                offsetEntityPreview.roundPoint = offsetEntityPreview.primitiveType == GL_POINTS;
                offsetEntityPreview.vertices = offsetVertices;
                primitives.push_back(std::move(offsetEntityPreview));
            }

            return primitives;
        }

        if (state.rotatePreviewActive && !selectedItems.isEmpty())
        {
            constexpr double kRadiansPerDegree = 3.14159265358979323846 / 180.0;
            const QVector3D basePoint = CadViewerUtils::flattenedToGroundPlane(state.rotatePreviewBasePoint);
            const QVector3D currentPoint = CadViewerUtils::flattenedToGroundPlane(state.currentPos);
            const double radians = state.rotatePreviewAngleDegrees * kRadiansPerDegree;

            for (CadItem* item : selectedItems)
            {
                if (item == nullptr || item->m_geometry.vertices.isEmpty())
                {
                    continue;
                }

                TransientPrimitive rotatedEntityPreview;
                rotatedEntityPreview.primitiveType = CadViewerUtils::primitiveTypeForEntity(item);
                rotatedEntityPreview.color = QVector3D(0.98f, 0.67f, 0.12f);
                rotatedEntityPreview.pointSize = rotatedEntityPreview.primitiveType == GL_POINTS ? 12.0f : 1.0f;
                rotatedEntityPreview.roundPoint = rotatedEntityPreview.primitiveType == GL_POINTS;
                rotatedEntityPreview.vertices.reserve(item->m_geometry.vertices.size());

                for (const QVector3D& vertex : item->m_geometry.vertices)
                {
                    const QVector3D planarVertex = CadViewerUtils::flattenedToGroundPlane(vertex);
                    rotatedEntityPreview.vertices.append(rotatePlanarPreviewPoint(planarVertex, basePoint, radians));
                }

                if (!rotatedEntityPreview.vertices.isEmpty())
                {
                    primitives.push_back(std::move(rotatedEntityPreview));
                }
            }

            TransientPrimitive guideLine;
            guideLine.primitiveType = GL_LINES;
            guideLine.color = QVector3D(0.35f, 0.90f, 1.0f);
            guideLine.vertices = { basePoint, currentPoint };
            primitives.push_back(std::move(guideLine));

            TransientPrimitive basePointMarker;
            basePointMarker.primitiveType = GL_POINTS;
            basePointMarker.color = QVector3D(1.0f, 0.92f, 0.25f);
            basePointMarker.pointSize = 11.0f;
            basePointMarker.roundPoint = true;
            basePointMarker.vertices = { basePoint };
            primitives.push_back(std::move(basePointMarker));

            return primitives;
        }

        if (state.scalePreviewActive && !selectedItems.isEmpty())
        {
            const QVector3D basePoint = CadViewerUtils::flattenedToGroundPlane(state.scalePreviewBasePoint);
            const QVector3D currentPoint = CadViewerUtils::flattenedToGroundPlane(state.currentPos);
            const double factor = std::max(state.scalePreviewFactor, 0.001);

            for (CadItem* item : selectedItems)
            {
                if (item == nullptr || item->m_geometry.vertices.isEmpty())
                {
                    continue;
                }

                TransientPrimitive scaledEntityPreview;
                scaledEntityPreview.primitiveType = CadViewerUtils::primitiveTypeForEntity(item);
                scaledEntityPreview.color = QVector3D(0.98f, 0.67f, 0.12f);
                scaledEntityPreview.pointSize = scaledEntityPreview.primitiveType == GL_POINTS ? 12.0f : 1.0f;
                scaledEntityPreview.roundPoint = scaledEntityPreview.primitiveType == GL_POINTS;
                scaledEntityPreview.vertices.reserve(item->m_geometry.vertices.size());

                for (const QVector3D& vertex : item->m_geometry.vertices)
                {
                    const QVector3D planarVertex = CadViewerUtils::flattenedToGroundPlane(vertex);
                    scaledEntityPreview.vertices.append(basePoint + (planarVertex - basePoint) * static_cast<float>(factor));
                }

                if (!scaledEntityPreview.vertices.isEmpty())
                {
                    primitives.push_back(std::move(scaledEntityPreview));
                }
            }

            TransientPrimitive guideLine;
            guideLine.primitiveType = GL_LINES;
            guideLine.color = QVector3D(0.35f, 0.90f, 1.0f);
            guideLine.vertices = { basePoint, currentPoint };
            primitives.push_back(std::move(guideLine));

            TransientPrimitive basePointMarker;
            basePointMarker.primitiveType = GL_POINTS;
            basePointMarker.color = QVector3D(1.0f, 0.92f, 0.25f);
            basePointMarker.pointSize = 11.0f;
            basePointMarker.roundPoint = true;
            basePointMarker.vertices = { basePoint };
            primitives.push_back(std::move(basePointMarker));

            return primitives;
        }

        // 处理各种绘制命令的预览
        switch (state.drawType)
        {
        case DrawType::Point:
        {
            // 点命令只显示一个跟随鼠标的预览点
            if (state.pointSubMode == PointDrawSubMode::AwaitPosition)
            {
                TransientPrimitive pointPreview;
                pointPreview.primitiveType = GL_POINTS;
                pointPreview.color = QVector3D(0.35f, 0.90f, 1.0f);  // 青色
                pointPreview.pointSize = 10.0f;
                pointPreview.roundPoint = true;
                pointPreview.vertices = { CadViewerUtils::flattenedToGroundPlane(state.currentPos) };
                primitives.push_back(std::move(pointPreview));
            }
            break;
        }
        case DrawType::Line:
        {
            // 直线命令在起点确定后显示一条橡皮筋线段
            if (state.lineSubMode == LineDrawSubMode::AwaitEndPoint && !state.commandPoints.isEmpty())
            {
                TransientPrimitive linePreview;
                linePreview.primitiveType = GL_LINES;
                linePreview.color = QVector3D(0.35f, 0.90f, 1.0f);
                linePreview.vertices =
                {
                    state.commandPoints.front(),
                    CadViewerUtils::flattenedToGroundPlane(state.currentPos)
                };
                primitives.push_back(std::move(linePreview));
            }
            break;
        }
        case DrawType::Xline:
        {
            if (state.lineSubMode == LineDrawSubMode::AwaitEndPoint && !state.commandPoints.isEmpty())
            {
                const QVector3D basePoint = CadViewerUtils::flattenedToGroundPlane(state.commandPoints.front());
                QVector3D direction = CadViewerUtils::flattenedToGroundPlane(state.currentPos) - basePoint;

                if (direction.lengthSquared() > 1.0e-10f)
                {
                    direction.normalize();

                    TransientPrimitive xlinePreview;
                    xlinePreview.primitiveType = GL_LINES;
                    xlinePreview.color = QVector3D(0.35f, 0.90f, 1.0f);
                    xlinePreview.vertices =
                    {
                        basePoint - direction * kXlinePreviewHalfLength,
                        basePoint + direction * kXlinePreviewHalfLength
                    };
                    primitives.push_back(std::move(xlinePreview));
                }
            }
            break;
        }
        case DrawType::Rectangle:
        {
            if (state.rectangleSubMode == RectangleDrawSubMode::AwaitSecondCorner && !state.commandPoints.isEmpty())
            {
                const QVector3D firstCorner = CadViewerUtils::flattenedToGroundPlane(state.commandPoints.front());
                const QVector3D currentPoint = CadViewerUtils::flattenedToGroundPlane(state.currentPos);

                TransientPrimitive rectanglePreview;
                rectanglePreview.primitiveType = GL_LINE_STRIP;
                rectanglePreview.color = QVector3D(0.35f, 0.90f, 1.0f);
                appendRectanglePreview(rectanglePreview.vertices, firstCorner, currentPoint);

                if (!rectanglePreview.vertices.isEmpty())
                {
                    primitives.push_back(std::move(rectanglePreview));
                }
            }
            break;
        }
        case DrawType::Polygon:
        {
            if (state.polygonSubMode == PolygonDrawSubMode::AwaitRadius && !state.commandPoints.isEmpty())
            {
                const QVector3D center = CadViewerUtils::flattenedToGroundPlane(state.commandPoints.front());
                const QVector3D currentPoint = CadViewerUtils::flattenedToGroundPlane(state.currentPos);
                const float referenceRadius = (currentPoint - center).length();

                TransientPrimitive radiusGuide;
                radiusGuide.primitiveType = GL_LINES;
                radiusGuide.color = QVector3D(0.35f, 0.90f, 1.0f);
                radiusGuide.vertices = { center, currentPoint };
                primitives.push_back(std::move(radiusGuide));

                TransientPrimitive circleGuide;
                circleGuide.primitiveType = GL_LINE_STRIP;
                circleGuide.color = QVector3D(0.22f, 0.56f, 0.82f);
                appendCirclePreview(circleGuide.vertices, center, referenceRadius);

                if (!circleGuide.vertices.isEmpty())
                {
                    primitives.push_back(std::move(circleGuide));
                }

                TransientPrimitive polygonPreview;
                polygonPreview.primitiveType = GL_LINE_STRIP;
                polygonPreview.color = QVector3D(0.35f, 0.90f, 1.0f);
                appendPolygonPreview
                (
                    polygonPreview.vertices,
                    center,
                    currentPoint,
                    state.polygonSideCount,
                    state.polygonCircumscribedAboutCircle
                );

                if (!polygonPreview.vertices.isEmpty())
                {
                    primitives.push_back(std::move(polygonPreview));
                }
            }
            break;
        }
        case DrawType::Circle:
        {
            // 圆命令同时显示半径引导线和圆轮廓预览
            if (state.circleSubMode == CircleDrawSubMode::AwaitRadius && !state.commandPoints.isEmpty())
            {
                const QVector3D center = state.commandPoints.front();
                const QVector3D currentPoint = CadViewerUtils::flattenedToGroundPlane(state.currentPos);
                const float radius = (currentPoint - center).length();

                // 半径引导线
                TransientPrimitive radiusLine;
                radiusLine.primitiveType = GL_LINES;
                radiusLine.color = QVector3D(0.35f, 0.90f, 1.0f);
                radiusLine.vertices = { center, currentPoint };
                primitives.push_back(std::move(radiusLine));

                // 圆轮廓预览
                TransientPrimitive circlePreview;
                circlePreview.primitiveType = GL_LINE_STRIP;
                circlePreview.color = QVector3D(0.35f, 0.90f, 1.0f);
                appendCirclePreview(circlePreview.vertices, center, radius);
                primitives.push_back(std::move(circlePreview));
            }
            break;
        }
        case DrawType::Arc:
        {
            // 圆弧命令按“圆心 -> 起点/半径 -> 终点”三步预览。
            if (state.arcSubMode == ArcDrawSubMode::AwaitRadius && !state.commandPoints.isEmpty())
            {
                // 圆心确定后仅显示半径方向虚线，不提前显示完整圆。
                TransientPrimitive radiusGuide;
                radiusGuide.primitiveType = GL_LINES;
                radiusGuide.color = QVector3D(0.35f, 0.90f, 1.0f);
                appendDashedLinePreview
                (
                    radiusGuide.vertices,
                    state.commandPoints.front(),
                    CadViewerUtils::flattenedToGroundPlane(state.currentPos)
                );

                if (!radiusGuide.vertices.isEmpty())
                {
                    primitives.push_back(std::move(radiusGuide));
                }
            }
            else if (state.arcSubMode == ArcDrawSubMode::AwaitEndAngle && state.commandPoints.size() >= 2)
            {
                const QVector3D center = state.commandPoints[0];
                const QVector3D startPoint = state.commandPoints[1];
                const QVector3D endPoint = projectPointToRadius
                (
                    center,
                    startPoint,
                    CadViewerUtils::flattenedToGroundPlane(state.currentPos)
                );
                const bool useComplementArc = (state.keyboardModifiers & Qt::ControlModifier) != 0;

                // 圆弧轮廓预览
                TransientPrimitive arcPreview;
                arcPreview.primitiveType = GL_LINE_STRIP;
                arcPreview.color = QVector3D(0.35f, 0.90f, 1.0f);
                appendArcPreview(arcPreview.vertices, center, startPoint, endPoint, useComplementArc);

                if (!arcPreview.vertices.isEmpty())
                {
                    primitives.push_back(std::move(arcPreview));
                }
            }
            break;
        }
        case DrawType::Ellipse:
        {
            if (state.ellipseSubMode == EllipseDrawSubMode::AwaitMajorAxis && !state.commandPoints.isEmpty())
            {
                // 长轴预览
                TransientPrimitive axisLine;
                axisLine.primitiveType = GL_LINES;
                axisLine.color = QVector3D(0.35f, 0.90f, 1.0f);
                axisLine.vertices =
                {
                    state.commandPoints.front(),
                    CadViewerUtils::flattenedToGroundPlane(state.currentPos)
                };
                primitives.push_back(std::move(axisLine));
            }
            else if (state.ellipseSubMode == EllipseDrawSubMode::AwaitMinorAxis && state.commandPoints.size() >= 2)
            {
                const QVector3D center = state.commandPoints[0];
                const QVector3D axisEnd = state.commandPoints[1];
                const QVector3D currentPoint = CadViewerUtils::flattenedToGroundPlane(state.currentPos);

                // 长轴预览
                TransientPrimitive axisLine;
                axisLine.primitiveType = GL_LINES;
                axisLine.color = QVector3D(0.35f, 0.90f, 1.0f);
                axisLine.vertices = { center, axisEnd };
                primitives.push_back(std::move(axisLine));

                // 椭圆轮廓预览
                TransientPrimitive ellipsePreview;
                ellipsePreview.primitiveType = GL_LINE_STRIP;
                ellipsePreview.color = QVector3D(0.35f, 0.90f, 1.0f);
                appendEllipsePreview(ellipsePreview.vertices, center, axisEnd, currentPoint);
                primitives.push_back(std::move(ellipsePreview));
            }
            break;
        }
        case DrawType::Polyline:
        case DrawType::LWPolyline:
        {
            if (!state.commandPoints.isEmpty())
            {
                TransientPrimitive polylinePreview;
                polylinePreview.primitiveType = GL_LINE_STRIP;
                polylinePreview.color = QVector3D(0.35f, 0.90f, 1.0f);

                // 添加起点
                polylinePreview.vertices.append(CadViewerUtils::flattenedToGroundPlane(state.commandPoints.front()));

                // 添加已确定的线段
                for (int i = 0; i + 1 < state.commandPoints.size(); ++i)
                {
                    const QVector3D segmentStart = CadViewerUtils::flattenedToGroundPlane(state.commandPoints[i]);
                    const QVector3D segmentEnd = CadViewerUtils::flattenedToGroundPlane(state.commandPoints[i + 1]);
                    const double bulge = i < state.commandBulges.size() ? state.commandBulges[i] : 0.0;
                    appendBulgePreview(polylinePreview.vertices, segmentStart, segmentEnd, bulge);
                }

                // 添加当前鼠标位置的预览线段
                const QVector3D currentPoint = CadViewerUtils::flattenedToGroundPlane(state.currentPos);
                const QVector3D lastPoint = CadViewerUtils::flattenedToGroundPlane(state.commandPoints.back());

                if ((currentPoint - lastPoint).lengthSquared() > 1.0e-10f)
                {
                    const bool arcMode = state.drawType == DrawType::Polyline
                        ? state.polylineSubMode == PolylineDrawSubMode::AwaitArcEndPoint
                        : state.lwPolylineSubMode == LWPolylineDrawSubMode::AwaitArcEndPoint;

                    if (arcMode && state.commandPoints.size() >= 2)
                    {
                        // 圆弧模式：计算续接圆弧
                        const QVector3D tangentDirection = polylineEndTangent(state.commandPoints, state.commandBulges);
                        const double previewBulge = bulgeFromTangent(lastPoint, tangentDirection, currentPoint);

                        if (std::isfinite(previewBulge))
                        {
                            appendBulgePreview(polylinePreview.vertices, lastPoint, currentPoint, previewBulge);
                        }
                        else
                        {
                            polylinePreview.vertices.append(currentPoint);
                        }
                    }
                    else
                    {
                        // 直线模式：直接连接
                        polylinePreview.vertices.append(currentPoint);
                    }
                }

                primitives.push_back(std::move(polylinePreview));
            }
            break;
        }
        default:
            break;
        }

        return primitives;
    }
}

