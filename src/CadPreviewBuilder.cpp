// CadPreviewBuilder 实现文件
// 实现 CadPreviewBuilder 模块，对应头文件中声明的主要行为和协作流程。
// 预览构建模块，负责根据当前命令状态生成 transient 预览图元。

#include "pch.h"

#include "CadPreviewBuilder.h"

// CAD 模块内部依赖
#include "CadItem.h"
#include "CadViewerUtils.h"
#include "DrawStateMachine.h"

// 标准库
#include <cmath>
#include <limits>

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
        int segments = kPreviewCurveSegments
    )
    {
        // 计算圆弧半径
        const float radius = (startPoint - center).length();

        if (radius <= 1.0e-6f)
        {
            return;
        }

        // 计算起始角度和终止角度
        float startAngle = std::atan2(startPoint.y() - center.y(), startPoint.x() - center.x());
        float endAngle = std::atan2(endPoint.y() - center.y(), endPoint.x() - center.x());

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
        CadItem* selectedItem
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
            if (selectedItem != nullptr
                && state.moveSubMode == MoveEditSubMode::AwaitTargetPoint
                && !state.commandPoints.isEmpty())
            {
                const QVector3D basePoint = state.commandPoints.front();
                const QVector3D delta = state.currentPos - basePoint;

                // 创建移动后的实体预览
                TransientPrimitive movedEntityPreview;
                movedEntityPreview.primitiveType = CadViewerUtils::primitiveTypeForEntity(selectedItem);
                movedEntityPreview.color = QVector3D(0.98f, 0.67f, 0.12f);  // 橙色
                movedEntityPreview.pointSize = movedEntityPreview.primitiveType == GL_POINTS ? 12.0f : 1.0f;
                movedEntityPreview.roundPoint = movedEntityPreview.primitiveType == GL_POINTS;

                movedEntityPreview.vertices.reserve(selectedItem->m_geometry.vertices.size());

                for (const QVector3D& vertex : selectedItem->m_geometry.vertices)
                {
                    movedEntityPreview.vertices.append(vertex + delta);
                }

                primitives.push_back(std::move(movedEntityPreview));

                // 创建引导线预览
                TransientPrimitive guideLine;
                guideLine.primitiveType = GL_LINES;
                guideLine.color = QVector3D(0.35f, 0.90f, 1.0f);  // 青色
                guideLine.vertices = { basePoint, state.currentPos };
                primitives.push_back(std::move(guideLine));
            }

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
            // 圆弧命令会随着子状态不同，分别显示半径线、起始半径线和弧线轮廓
            if (state.arcSubMode == ArcDrawSubMode::AwaitRadius && !state.commandPoints.isEmpty())
            {
                // 半径线预览
                TransientPrimitive radiusLine;
                radiusLine.primitiveType = GL_LINES;
                radiusLine.color = QVector3D(0.35f, 0.90f, 1.0f);
                radiusLine.vertices =
                {
                    state.commandPoints.front(),
                    CadViewerUtils::flattenedToGroundPlane(state.currentPos)
                };
                primitives.push_back(std::move(radiusLine));
            }
            else if (state.arcSubMode == ArcDrawSubMode::AwaitStartAngle && state.commandPoints.size() >= 2)
            {
                const QVector3D center = state.commandPoints[0];
                const QVector3D radiusPoint = state.commandPoints[1];
                const QVector3D startPoint = projectPointToRadius
                (
                    center,
                    radiusPoint,
                    CadViewerUtils::flattenedToGroundPlane(state.currentPos)
                );

                // 完整圆轮廓预览
                TransientPrimitive circlePreview;
                circlePreview.primitiveType = GL_LINE_STRIP;
                circlePreview.color = QVector3D(0.35f, 0.90f, 1.0f);
                appendCirclePreview(circlePreview.vertices, center, (radiusPoint - center).length());
                primitives.push_back(std::move(circlePreview));

                // 起始半径线
                TransientPrimitive startLine;
                startLine.primitiveType = GL_LINES;
                startLine.color = QVector3D(0.35f, 0.90f, 1.0f);
                startLine.vertices = { center, startPoint };
                primitives.push_back(std::move(startLine));
            }
            else if (state.arcSubMode == ArcDrawSubMode::AwaitEndAngle && state.commandPoints.size() >= 3)
            {
                const QVector3D center = state.commandPoints[0];
                const QVector3D radiusPoint = state.commandPoints[1];
                const QVector3D startPoint = projectPointToRadius(center, radiusPoint, state.commandPoints[2]);
                const QVector3D endPoint = projectPointToRadius
                (
                    center,
                    radiusPoint,
                    CadViewerUtils::flattenedToGroundPlane(state.currentPos)
                );

                // 圆弧轮廓预览
                TransientPrimitive arcPreview;
                arcPreview.primitiveType = GL_LINE_STRIP;
                arcPreview.color = QVector3D(0.35f, 0.90f, 1.0f);
                appendArcPreview(arcPreview.vertices, center, startPoint, endPoint);
                primitives.push_back(std::move(arcPreview));
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