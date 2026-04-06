#include "pch.h"

#include "CadViewer.h"

#include "CadDocument.h"
#include "CadItem.h"

#include <QDragEnterEvent>
#include <QKeyEvent>
#include <QMimeData>
#include <QMatrix3x3>
#include <QOpenGLContext>
#include <QQuaternion>
#include <QSurfaceFormat>
#include <QVector4D>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
    // 常用数学常量。
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kDegToRad = kPi / 180.0f;

    // 屏幕空间拾取框半尺寸（像素）。
    // 该值同时用于屏幕空间拾取判定和十字线中心方框绘制，确保视觉与交互一致。
    constexpr float kPickBoxHalfSizePixels = 10.0f;

    // 十字线与拾取框颜色。
    const QVector3D kCrosshairColor(0.38f, 0.88f, 1.0f);

    // 十字线固定半长（世界坐标）。
    constexpr float kCrosshairHalfLengthWorld = 100000.0f;

    // 判断向量是否退化为零向量的阈值。
    constexpr float kBasisEpsilon = 1.0e-8f;

    // 极区限制半角。
    // 当视线 forward 过于接近世界 Z 轴正/负方向时，禁止继续旋转，
    // 用于避免在“几乎竖直向上/向下”时交互不稳定。
    constexpr float kPoleConeHalfAngleDeg = 0.01f;
    const float kPoleConeDot = std::cos(kPoleConeHalfAngleDeg * kDegToRad);

    // 世界坐标系中的参考方向。
    const QVector3D kWorldUp(0.0f, 0.0f, 1.0f);
    const QVector3D kWorldDown(0.0f, 0.0f, -1.0f);
    const QVector3D kNorthUp(0.0f, 1.0f, 0.0f);

    // 相机局部坐标系约定：
    // forward = -Z, right = +X, up = +Y
    const QVector3D kLocalForward(0.0f, 0.0f, -1.0f);
    const QVector3D kLocalRight(1.0f, 0.0f, 0.0f);
    const QVector3D kLocalUp(0.0f, 1.0f, 0.0f);

    // 构造 basis 时 right 退化时使用的兜底方向。
    const QVector3D kFallbackRight(1.0f, 0.0f, 0.0f);

    // 将实体指针转为稳定 ID。
    // 当前实现直接使用对象地址做 key，便于缓存其 GPU 资源。
    EntityId toEntityId(const CadItem* entity)
    {
        return static_cast<EntityId>(reinterpret_cast<quintptr>(entity));
    }

    // 对向量做归一化；若长度过小则返回 fallback。
    QVector3D normalizedOr(const QVector3D& vector, const QVector3D& fallback)
    {
        if (vector.lengthSquared() <= kBasisEpsilon)
        {
            return fallback;
        }

        QVector3D normalized = vector;
        normalized.normalize();
        return normalized;
    }

    // 描述一个完整的相机正交基。
    struct CameraBasis
    {
        QVector3D forward;
        QVector3D right;
        QVector3D up;
    };

    // 将世界坐标点投影到屏幕坐标。
    // 输入是 VP 矩阵和视口尺寸，输出 Qt 屏幕坐标（左上为原点）。
    QPointF projectToScreen
    (
        const QVector3D& worldPos,
        const QMatrix4x4& viewProjection,
        int viewportWidth,
        int viewportHeight
    )
    {
        const QVector4D clip = viewProjection * QVector4D(worldPos, 1.0f);

        // 裁剪空间 w 为 0 时无法做透视除法，返回空点。
        if (qFuzzyIsNull(clip.w()))
        {
            return QPointF();
        }

        // 转为 NDC（-1~1）。
        const QVector3D ndc = clip.toVector3DAffine();

        // NDC -> 屏幕坐标。
        return QPointF
        (
            (ndc.x() + 1.0f) * 0.5f * viewportWidth,
            (1.0f - ndc.y()) * 0.5f * viewportHeight
        );
    }

    // 计算点到线段的最短距离平方。
    // 用于屏幕空间实体拾取。
    float distanceToSegmentSquared(const QPointF& point, const QPointF& start, const QPointF& end)
    {
        const QPointF segment = end - start;
        const double lengthSquared = segment.x() * segment.x() + segment.y() * segment.y();

        // 线段退化为点。
        if (lengthSquared <= 1.0e-12)
        {
            const QPointF delta = point - start;
            return static_cast<float>(delta.x() * delta.x() + delta.y() * delta.y());
        }

        const QPointF fromStart = point - start;
        const double t = std::clamp
        (
            (fromStart.x() * segment.x() + fromStart.y() * segment.y()) / lengthSquared,
            0.0,
            1.0
        );

        const QPointF projection = start + segment * t;
        const QPointF delta = point - projection;
        return static_cast<float>(delta.x() * delta.x() + delta.y() * delta.y());
    }

    // 根据 forward 和 up 提示方向构造一个稳定的相机基。
    // 若 preferredUp 与 forward 共线，则退化切换到 fallbackUp。
    CameraBasis buildCameraBasis(const QVector3D& forward, const QVector3D& preferredUp, const QVector3D& fallbackUp)
    {
        CameraBasis basis;
        basis.forward = normalizedOr(forward, kLocalForward);

        QVector3D upHint = preferredUp;
        QVector3D right = QVector3D::crossProduct(basis.forward, upHint);

        // preferredUp 退化时切换到备用 up。
        if (right.lengthSquared() <= kBasisEpsilon)
        {
            upHint = fallbackUp;
            right = QVector3D::crossProduct(basis.forward, upHint);
        }

        basis.right = normalizedOr(right, kFallbackRight);
        basis.up = normalizedOr(QVector3D::crossProduct(basis.right, basis.forward), kLocalUp);
        return basis;
    }

    // 将相机基转为四元数朝向。
    // 这里矩阵列向量分别对应 right / up / backward。
    QQuaternion quaternionFromBasis(const CameraBasis& basis)
    {
        const QVector3D backward = -basis.forward;

        QMatrix3x3 matrix;
        matrix(0, 0) = basis.right.x();
        matrix(1, 0) = basis.right.y();
        matrix(2, 0) = basis.right.z();
        matrix(0, 1) = basis.up.x();
        matrix(1, 1) = basis.up.y();
        matrix(2, 1) = basis.up.z();
        matrix(0, 2) = backward.x();
        matrix(1, 2) = backward.y();
        matrix(2, 2) = backward.z();

        QQuaternion orientation = QQuaternion::fromRotationMatrix(matrix);

        if (orientation.isNull())
        {
            return QQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
        }

        orientation.normalize();
        return orientation;
    }

    // 四元数点积。
    // 可用于判断两个旋转是否位于单位四维球的同一半球。
    float quaternionDot(const QQuaternion& lhs, const QQuaternion& rhs)
    {
        return lhs.scalar() * rhs.scalar()
            + lhs.x() * rhs.x()
            + lhs.y() * rhs.y()
            + lhs.z() * rhs.z();
    }

    // 统一四元数半球，避免 q / -q 表示同一旋转时出现数值不连续。
    // 常用于交互过程中的平滑连续更新。
    QQuaternion alignQuaternionHemisphere(const QQuaternion& previous, const QQuaternion& current)
    {
        QQuaternion normalizedPrevious = previous;
        QQuaternion normalizedCurrent = current;

        if (!normalizedPrevious.isNull())
        {
            normalizedPrevious.normalize();
        }

        if (!normalizedCurrent.isNull())
        {
            normalizedCurrent.normalize();
        }

        if (!normalizedPrevious.isNull() && !normalizedCurrent.isNull() && quaternionDot(normalizedPrevious, normalizedCurrent) < 0.0f)
        {
            return QQuaternion
            (
                -normalizedCurrent.scalar(),
                -normalizedCurrent.x(),
                -normalizedCurrent.y(),
                -normalizedCurrent.z()
            );
        }

        return normalizedCurrent;
    }

    // 将任意四元数标准化；若为空则返回单位四元数。
    QQuaternion normalizedQuaternionOrIdentity(const QQuaternion& quaternion)
    {
        QQuaternion normalized = quaternion;

        if (normalized.isNull())
        {
            return QQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
        }

        normalized.normalize();
        return normalized;
    }

    // 判断给定朝向是否进入“极区”。
    // 极区定义为 forward 与世界上方向过于接近（向上或向下）。
    bool violatesViewConstraint(const QQuaternion& orientation)
    {
        const QQuaternion q = normalizedQuaternionOrIdentity(orientation);
        const QVector3D forward = normalizedOr(q.rotatedVector(kLocalForward), kLocalForward);

        const float forwardUpDot = QVector3D::dotProduct(forward, kWorldUp);

        return forwardUpDot >= kPoleConeDot || forwardUpDot <= -kPoleConeDot;
    }

    // 根据实体类型选择 OpenGL 图元类型。
    GLenum primitiveTypeForEntity(const CadItem* entity)
    {
        if (entity == nullptr)
        {
            return GL_LINE_STRIP;
        }

        switch (entity->m_type)
        {
        case DRW::ETYPE::POINT:
            return GL_POINTS;
        case DRW::ETYPE::LINE:
            return GL_LINES;
        default:
            return GL_LINE_STRIP;
        }
    }

    QVector3D flattenedToGroundPlane(const QVector3D& point)
    {
        return QVector3D(point.x(), point.y(), 0.0f);
    }

    void appendCirclePreview(QVector<QVector3D>& vertices, const QVector3D& center, float radius, int segments = 96)
    {
        if (radius <= 1.0e-6f)
        {
            return;
        }

        vertices.reserve(segments + 1);

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

    QVector3D projectPointToRadius(const QVector3D& center, const QVector3D& radiusPoint, const QVector3D& currentPoint)
    {
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

    void appendArcPreview
    (
        QVector<QVector3D>& vertices,
        const QVector3D& center,
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        int segments = 96
    )
    {
        const float radius = (startPoint - center).length();

        if (radius <= 1.0e-6f)
        {
            return;
        }

        float startAngle = std::atan2(startPoint.y() - center.y(), startPoint.x() - center.x());
        float endAngle = std::atan2(endPoint.y() - center.y(), endPoint.x() - center.x());

        while (endAngle <= startAngle)
        {
            endAngle += 2.0f * kPi;
        }

        vertices.reserve(segments + 1);

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

    void appendEllipsePreview
    (
        QVector<QVector3D>& vertices,
        const QVector3D& center,
        const QVector3D& majorAxisPoint,
        const QVector3D& ratioPoint,
        int segments = 96
    )
    {
        const QVector3D majorAxis = majorAxisPoint - center;
        const float majorLength = majorAxis.length();

        if (majorLength <= 1.0e-6f)
        {
            return;
        }

        const QVector3D toRatioPoint = ratioPoint - center;
        const float projectedLength = QVector3D::dotProduct(toRatioPoint, majorAxis) / majorLength;
        const float minorSquared = std::max(0.0f, toRatioPoint.lengthSquared() - projectedLength * projectedLength);
        const float minorLength = std::sqrt(minorSquared);

        if (minorLength <= 1.0e-6f)
        {
            return;
        }

        QVector3D majorDirection = majorAxis;
        majorDirection.normalize();
        const QVector3D minorDirection(-majorDirection.y(), majorDirection.x(), 0.0f);

        vertices.reserve(segments + 1);

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

    void appendBulgePreview(QVector<QVector3D>& vertices, const QVector3D& startPoint, const QVector3D& endPoint, double bulge)
    {
        const double dx = endPoint.x() - startPoint.x();
        const double dy = endPoint.y() - startPoint.y();
        const double chordLength = std::sqrt(dx * dx + dy * dy);

        if (chordLength <= 1.0e-10 || std::abs(bulge) < 1.0e-8)
        {
            vertices.append(endPoint);
            return;
        }

        const double midpointX = (startPoint.x() + endPoint.x()) * 0.5;
        const double midpointY = (startPoint.y() + endPoint.y()) * 0.5;
        const double centerOffset = chordLength * (1.0 / bulge - bulge) * 0.25;
        const double centerX = midpointX - centerOffset * (dy / chordLength);
        const double centerY = midpointY + centerOffset * (dx / chordLength);
        const double radius = std::hypot(startPoint.x() - centerX, startPoint.y() - centerY);
        const double startAngle = std::atan2(startPoint.y() - centerY, startPoint.x() - centerX);
        const double sweepAngle = 4.0 * std::atan(bulge);
        const int segments = std::max(4, static_cast<int>(std::ceil(std::abs(sweepAngle) / (2.0 * kPi) * 128.0)));

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

    QVector3D polylineEndTangent(const QVector<QVector3D>& points, const QVector<double>& bulges)
    {
        if (points.size() < 2)
        {
            return QVector3D();
        }

        const QVector3D startPoint = flattenedToGroundPlane(points[points.size() - 2]);
        const QVector3D endPoint = flattenedToGroundPlane(points.back());
        const double bulge = bulges.size() >= points.size() - 1 ? bulges[points.size() - 2] : 0.0;

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

        const QVector3D radiusVector = endPoint - center;
        const QVector3D tangent = bulge > 0.0
            ? QVector3D(-radiusVector.y(), radiusVector.x(), 0.0f)
            : QVector3D(radiusVector.y(), -radiusVector.x(), 0.0f);

        return normalizedOrZero(tangent);
    }

    double bulgeFromTangent(const QVector3D& startPoint, const QVector3D& tangentDirection, const QVector3D& endPoint)
    {
        const QVector3D planarTangent = normalizedOrZero(QVector3D(tangentDirection.x(), tangentDirection.y(), 0.0f));
        const QVector3D chordVector = flattenedToGroundPlane(endPoint) - flattenedToGroundPlane(startPoint);

        if (planarTangent.lengthSquared() <= 1.0e-10 || chordVector.lengthSquared() <= 1.0e-10)
        {
            return 0.0;
        }

        const double dotValue = QVector3D::dotProduct(planarTangent, chordVector);
        const double crossValue = planarTangent.x() * chordVector.y() - planarTangent.y() * chordVector.x();
        const double alpha = std::atan2(crossValue, dotValue);

        if (std::abs(std::abs(alpha) - kPi) <= 1.0e-6)
        {
            return std::numeric_limits<double>::infinity();
        }

        return std::tan(alpha * 0.5);
    }
}

// 由 target、forward 和 distance 计算相机眼点。
QVector3D OrbitalCamera::eyePosition() const
{
    return target - forwardDirection() * distance;
}

// 由 orientation 推导当前相机前方向。
QVector3D OrbitalCamera::forwardDirection() const
{
    return normalizedOr(normalizedQuaternionOrIdentity(orientation).rotatedVector(kLocalForward), kLocalForward);
}

// 由 orientation 推导当前相机右方向。
QVector3D OrbitalCamera::rightDirection() const
{
    return normalizedOr(normalizedQuaternionOrIdentity(orientation).rotatedVector(kLocalRight), kLocalRight);
}

// 由 orientation 推导当前相机上方向。
QVector3D OrbitalCamera::upDirection() const
{
    return normalizedOr(normalizedQuaternionOrIdentity(orientation).rotatedVector(kLocalUp), kLocalUp);
}

// 构造观察矩阵。
// 使用 eyePosition 作为相机位置，target 作为观察中心，upDirection 作为相机上方向。
QMatrix4x4 OrbitalCamera::viewMatrix() const
{
    QMatrix4x4 matrix;
    matrix.lookAt(eyePosition(), target, upDirection());
    return matrix;
}

// 构造正交投影矩阵。
// viewHeight 决定视口高度，宽度由 aspectRatio 推导。
QMatrix4x4 OrbitalCamera::projectionMatrix(float aspectRatio) const
{
    const float safeAspectRatio = std::max(0.001f, aspectRatio);
    const float halfHeight = viewHeight * 0.5f;
    const float halfWidth = halfHeight * safeAspectRatio;

    QMatrix4x4 matrix;
    matrix.ortho(-halfWidth, halfWidth, -halfHeight, halfHeight, nearPlane, farPlane);
    return matrix;
}

// 返回 VP 矩阵，供渲染和坐标变换使用。
QMatrix4x4 OrbitalCamera::viewProjectionMatrix(float aspectRatio) const
{
    return projectionMatrix(aspectRatio) * viewMatrix();
}

// 根据当前 forward 重新构造一个稳定朝向。
// 当前版本保留该辅助函数，但实时轨道旋转中未使用。
QQuaternion stabilizedOrientationFromForward(const QQuaternion& orientation)
{
    const QQuaternion q = normalizedQuaternionOrIdentity(orientation);
    const QVector3D forward = normalizedOr(q.rotatedVector(kLocalForward), kLocalForward);

    // 用世界 Z 作为首选 up，给当前 forward 构造一套稳定的相机基。
    return quaternionFromBasis(buildCameraBasis(forward, kWorldUp, kNorthUp));
}

// 普通轨道旋转：
// - 水平旋转绕世界 Z 轴
// - 俯仰旋转绕当前相机右轴
// 并在极区前做约束，避免接近竖直时视角不稳定。
void OrbitalCamera::orbit(float deltaAzimuth, float deltaElevation)
{
    const QQuaternion previousOrientation = normalizedQuaternionOrIdentity(orientation);
    QQuaternion candidateOrientation = previousOrientation;

    // yaw：围绕世界上方向旋转。
    if (!qFuzzyIsNull(deltaAzimuth))
    {
        QQuaternion yawOrientation =
            QQuaternion::fromAxisAndAngle(kWorldUp, deltaAzimuth) * candidateOrientation;
        yawOrientation.normalize();

        if (!violatesViewConstraint(yawOrientation))
        {
            candidateOrientation = yawOrientation;
        }
    }

    // pitch：围绕当前相机右方向旋转。
    if (!qFuzzyIsNull(deltaElevation))
    {
        const QVector3D pitchAxis =
            normalizedOr(candidateOrientation.rotatedVector(kLocalRight), kLocalRight);

        QQuaternion pitchOrientation =
            QQuaternion::fromAxisAndAngle(pitchAxis, deltaElevation) * candidateOrientation;
        pitchOrientation.normalize();

        if (!violatesViewConstraint(pitchOrientation))
        {
            candidateOrientation = pitchOrientation;
        }
    }

    candidateOrientation.normalize();
    orientation = alignQuaternionHemisphere(previousOrientation, candidateOrientation);
    updateAxesSwappedState();
}

// 平移观察中心：
// 鼠标水平位移映射到相机右方向，垂直位移映射到相机上方向。
void OrbitalCamera::pan(float worldDx, float worldDy)
{
    target += rightDirection() * worldDx + upDirection() * worldDy;
}

// 缩放正交视口。
// factor > 1 表示放大，< 1 表示缩小。
void OrbitalCamera::zoom(float factor)
{
    if (factor <= 0.0f)
    {
        return;
    }

    viewHeight = std::clamp(viewHeight / factor, kMinViewHeight, kMaxViewHeight);
}

// 以指定世界锚点为基准缩放。
// 思路：
// 1. 先将 worldAnchor 投影到相机观察平面
// 2. 按缩放比调整 target，使锚点在屏幕中的视觉位置尽量保持稳定
void OrbitalCamera::zoomAtPoint(float factor, const QVector3D& worldAnchor)
{
    if (factor <= 0.0f)
    {
        return;
    }

    const QVector3D planeOffset = worldAnchor - target;
    const QVector3D forward = forwardDirection();
    const QVector3D anchorOnCameraPlane = planeOffset - QVector3D::dotProduct(planeOffset, forward) * forward;
    const QVector3D deltaTarget = anchorOnCameraPlane * (1.0f - 1.0f / factor);

    zoom(factor);
    target += deltaTarget;
}

// 根据场景包围盒适配视图。
// 当前实现统一回到二维顶视图，并按 XY 尺寸估算 viewHeight。
void OrbitalCamera::fitAll(const QVector3D& sceneMin, const QVector3D& sceneMax, float aspectRatio)
{
    const QVector3D size = sceneMax - sceneMin;

    // 目标点取场景中心。
    target = (sceneMin + sceneMax) * 0.5f;

    const float safeAspectRatio = std::max(0.001f, aspectRatio);
    const float width = std::max(size.x(), 1.0f);
    const float height = std::max(size.y(), 1.0f);
    const float depth = std::max(size.z(), 1.0f);

    // 正交高度按 XY 包围盒估算。
    viewHeight = std::clamp(std::max(height, width / safeAspectRatio) * 1.2f, kMinViewHeight, kMaxViewHeight);

    // distance 主要影响 eyePosition 和 orbit 手感。
    distance = std::max({ width, height, depth, 100.0f }) * 2.0f;

    resetTo2DTopView();
}

// 重置到二维顶视图。
// 约定为从 +Z 看向 XY 平面，屏幕上方向对齐 +Y。
void OrbitalCamera::resetTo2DTopView()
{
    orientation = quaternionFromBasis(buildCameraBasis(kWorldDown, kNorthUp, kFallbackRight));
    updateAxesSwappedState();
}

// 从 2D 顶视图进入默认 3D 视图。
// 当前实现只加一个固定 pitch，yaw 先保留为 0。
void OrbitalCamera::enter3DFrom2D()
{
    const QQuaternion topViewOrientation = quaternionFromBasis(buildCameraBasis(kWorldDown, kNorthUp, kFallbackRight));
    const QQuaternion yawOffset = QQuaternion::fromAxisAndAngle(kWorldUp, 0.0f);
    const QQuaternion pitchOffset = QQuaternion::fromAxisAndAngle(QVector3D(1.0f, 0.0f, 0.0f), 0.0f);
    orientation = yawOffset * pitchOffset * topViewOrientation;
    orientation.normalize();
    updateAxesSwappedState();
}

// 当视线穿过水平面后，forward.z() 为正，说明相机已处于“从下往上看”的状态。
// 此时可切换 Z 轴为虚线，以给用户视觉提示。
void OrbitalCamera::updateAxesSwappedState()
{
    m_axesSwapped = forwardDirection().z() > 0.0f;
}

// 构造函数：
// 配置 OpenGL 4.5 Core Profile，并启用鼠标跟踪与键盘焦点。
CadViewer::CadViewer(QWidget* parent)
    : QOpenGLWidget(parent)
{
    QSurfaceFormat surfaceFormat = format();
    surfaceFormat.setRenderableType(QSurfaceFormat::OpenGL);
    surfaceFormat.setVersion(4, 5);
    surfaceFormat.setProfile(QSurfaceFormat::CoreProfile);
    surfaceFormat.setDepthBufferSize(24);
    surfaceFormat.setStencilBufferSize(8);
    setFormat(surfaceFormat);

    setMouseTracking(true);
    setAcceptDrops(true);
    setFocusPolicy(Qt::StrongFocus);
    setCursor(Qt::BlankCursor);

    m_controller.setViewer(this);
}

// 析构时释放 OpenGL 资源。
// 需要先 makeCurrent()，确保当前上下文有效。
CadViewer::~CadViewer()
{
    if (context() != nullptr)
    {
        makeCurrent();
        clearAllBuffers();
        m_gridVao.destroy();
        m_gridVbo.destroy();
        m_axisVao.destroy();
        m_axisVbo.destroy();
        m_orbitMarkerVao.destroy();
        m_orbitMarkerVbo.destroy();
        m_transientVao.destroy();
        m_transientVbo.destroy();
        doneCurrent();
    }
}

// 设置当前文档：
// - 清除选择状态
// - 标记缓冲需重建
// - 更新场景包围盒
// - 若 OpenGL 已初始化则立即重建 GPU 缓冲
// - 最后执行 fitScene()
void CadViewer::setDocument(CadDocument* document)
{
    if (m_scene != nullptr)
    {
        disconnect(m_sceneChangedConnection);
    }

    m_scene = document;
    m_selectedEntityId = 0;
    m_buffersDirty = true;

    if (m_scene != nullptr)
    {
        m_sceneChangedConnection = connect
        (
            m_scene,
            &CadDocument::sceneChanged,
            this,
            &CadViewer::handleDocumentSceneChanged
        );
    }

    updateSceneBounds();

    if (m_glInitialized)
    {
        makeCurrent();
        rebuildAllBuffers();
        doneCurrent();
    }

    fitScene();
    update();
}

void CadViewer::setEditer(CadEditer* editer)
{
    m_editer = editer;
    m_controller.setEditer(editer);
}

// 适配整个场景，并回到 2D 平面视图。
void CadViewer::fitScene()
{
    updateSceneBounds();

    if (!m_hasSceneBounds)
    {
        return;
    }

    m_camera.fitAll(m_sceneMin, m_sceneMax, aspectRatio());
    m_viewMode = CameraViewMode::Planar2D;
    m_interactionMode = ViewInteractionMode::Idle;
    m_ignoreNextOrbitDelta = false;
    m_controller.reset();
    update();
}

void CadViewer::beginOrbitInteraction()
{
    if (m_viewMode == CameraViewMode::Planar2D)
    {
        m_camera.enter3DFrom2D();
        m_viewMode = CameraViewMode::Orbit3D;
        m_ignoreNextOrbitDelta = true;
    }
    else
    {
        m_ignoreNextOrbitDelta = false;
    }

    m_interactionMode = ViewInteractionMode::Orbiting;
    update();
}

void CadViewer::beginPanInteraction()
{
    m_interactionMode = ViewInteractionMode::Panning;
    m_ignoreNextOrbitDelta = false;
    update();
}

void CadViewer::updateOrbitInteraction(const QPoint& screenDelta)
{
    orbitCameraAroundSceneCenter(-screenDelta.x() * 0.4f, -screenDelta.y() * 0.4f);
    update();
}

void CadViewer::updatePanInteraction(const QPoint& screenDelta)
{
    m_camera.pan(-screenDelta.x() * pixelToWorldScale(), screenDelta.y() * pixelToWorldScale());
    update();
}

void CadViewer::endViewInteraction()
{
    m_interactionMode = ViewInteractionMode::Idle;
    m_ignoreNextOrbitDelta = false;
}

void CadViewer::selectEntityAt(const QPoint& screenPos)
{
    m_selectedEntityId = pickEntity(screenPos);
    update();
}

void CadViewer::zoomAtScreenPosition(const QPoint& screenPos, float factor)
{
    const QVector3D nearPoint = screenToWorld(screenPos, -1.0f);
    const QVector3D farPoint = screenToWorld(screenPos, 1.0f);
    const QVector3D rayDirection = farPoint - nearPoint;
    const QVector3D planeNormal = m_camera.forwardDirection();

    QVector3D anchor = m_camera.target;
    const float denominator = QVector3D::dotProduct(rayDirection, planeNormal);

    if (!qFuzzyIsNull(denominator))
    {
        const float t = QVector3D::dotProduct(m_camera.target - nearPoint, planeNormal) / denominator;
        anchor = nearPoint + rayDirection * t;
    }

    m_camera.zoomAtPoint(factor, anchor);
    update();
}

void CadViewer::resetToTopView()
{
    m_camera.resetTo2DTopView();
    m_viewMode = CameraViewMode::Planar2D;
    m_interactionMode = ViewInteractionMode::Idle;
    m_ignoreNextOrbitDelta = false;
    update();
}

void CadViewer::fitSceneView()
{
    fitScene();
}

CameraViewMode CadViewer::viewMode() const
{
    return m_viewMode;
}

ViewInteractionMode CadViewer::interactionMode() const
{
    return m_interactionMode;
}

bool CadViewer::shouldIgnoreNextOrbitDelta() const
{
    return m_ignoreNextOrbitDelta;
}

void CadViewer::consumeIgnoreNextOrbitDelta()
{
    m_ignoreNextOrbitDelta = false;
}

void CadViewer::requestViewUpdate()
{
    update();
}

CadItem* CadViewer::selectedEntity() const
{
    return findEntityById(m_selectedEntityId);
}

void CadViewer::appendCommandMessage(const QString& message)
{
    if (!message.trimmed().isEmpty())
    {
        emit commandMessageAppended(message.trimmed());
    }
}

void CadViewer::refreshCommandPrompt()
{
    emit commandPromptChanged(m_controller.currentPrompt());
}

// 放大视图。
void CadViewer::zoomIn(float factor)
{
    m_camera.zoom(factor);
    update();
}

// 缩小视图。
void CadViewer::zoomOut(float factor)
{
    if (factor <= 0.0f)
    {
        return;
    }

    m_camera.zoom(1.0f / factor);
    update();
}

// 屏幕坐标 -> 世界坐标。
// depth 取值使用 NDC 空间：-1 近平面，+1 远平面。
QVector3D CadViewer::screenToWorld(const QPoint& screenPos, float depth) const
{
    const float x = (2.0f * static_cast<float>(screenPos.x()) / std::max(1, m_viewportWidth)) - 1.0f;
    const float y = 1.0f - (2.0f * static_cast<float>(screenPos.y()) / std::max(1, m_viewportHeight));

    const QMatrix4x4 inverse = m_camera.viewProjectionMatrix(aspectRatio()).inverted();
    const QVector4D world = inverse * QVector4D(x, y, depth, 1.0f);

    if (qFuzzyIsNull(world.w()))
    {
        return QVector3D();
    }

    return world.toVector3DAffine();
}

QVector3D CadViewer::screenToGroundPlane(const QPoint& screenPos) const
{
    const QVector3D nearPoint = screenToWorld(screenPos, -1.0f);
    const QVector3D farPoint = screenToWorld(screenPos, 1.0f);
    const QVector3D rayDirection = farPoint - nearPoint;

    if (!qFuzzyIsNull(rayDirection.z()))
    {
        const float t = -nearPoint.z() / rayDirection.z();
        return nearPoint + rayDirection * t;
    }

    return flattenedToGroundPlane(nearPoint);
}

// 世界坐标 -> 屏幕坐标。
QPoint CadViewer::worldToScreen(const QVector3D& worldPos) const
{
    const QPointF screenPoint = projectToScreen
    (
        worldPos,
        m_camera.viewProjectionMatrix(aspectRatio()),
        m_viewportWidth,
        m_viewportHeight
    );

    return screenPoint.toPoint();
}

// OpenGL 初始化：
// - 初始化函数表
// - 设置基本渲染状态
// - 创建 shader / 网格 / 轴 / 轨道标记缓冲
// - 重建实体缓冲并适配场景
void CadViewer::initializeGL()
{
    initializeOpenGLFunctions();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glClearColor(0.08f, 0.09f, 0.11f, 1.0f);

    initShaders();
    initGridBuffer();
    initAxisBuffer();
    initOrbitMarkerBuffer();
    initTransientBuffer();

    m_glInitialized = true;
    rebuildAllBuffers();
    fitScene();
    refreshCommandPrompt();
}

// Qt 回调的逻辑尺寸变化。
// 实际绘制时 paintGL 还会按 devicePixelRatioF() 换算 framebuffer 尺寸。
void CadViewer::resizeGL(int w, int h)
{
    m_viewportWidth = std::max(1, w);
    m_viewportHeight = std::max(1, h);
}

// 主绘制入口：
// 1. 设置视口
// 2. 清屏
// 3. 若缓冲脏则重建
// 4. 绘制网格、实体、坐标轴、轨道中心标记
void CadViewer::paintGL()
{
    const int framebufferWidth = std::max(1, static_cast<int>(std::round(width() * devicePixelRatioF())));
    const int framebufferHeight = std::max(1, static_cast<int>(std::round(height() * devicePixelRatioF())));

    glViewport(0, 0, framebufferWidth, framebufferHeight);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (!m_glInitialized)
    {
        return;
    }

    if (m_buffersDirty)
    {
        rebuildAllBuffers();
    }

    renderGrid();
    renderEntities();
    renderTransientPrimitives();
    renderAxis();
    renderOrbitMarker();
}

// 鼠标按下：
// - 中键 + Shift：进入轨道旋转
// - 中键：进入平移
// - 左键：执行拾取
void CadViewer::mousePressEvent(QMouseEvent* event)
{
    m_cursorScreenPos = event->pos();
    m_showCrosshairOverlay = rect().contains(event->pos());

    if (!m_controller.handleMousePress(event))
    {
        QOpenGLWidget::mousePressEvent(event);
    }

    updateHoveredWorldPosition(event->pos());
    refreshCommandPrompt();
    update();
}

// 鼠标移动：
// - 平移模式：按像素位移换算为世界位移
// - 旋转模式：围绕场景中心轨道旋转
void CadViewer::mouseMoveEvent(QMouseEvent* event)
{
    m_cursorScreenPos = event->pos();
    m_showCrosshairOverlay = rect().contains(event->pos());

    if (!m_controller.handleMouseMove(event))
    {
        QOpenGLWidget::mouseMoveEvent(event);
    }

    updateHoveredWorldPosition(event->pos());
    update();
}

void CadViewer::leaveEvent(QEvent* event)
{
    m_showCrosshairOverlay = false;
    update();
    QOpenGLWidget::leaveEvent(event);
}

// 鼠标释放中键后退出当前交互模式。
void CadViewer::mouseReleaseEvent(QMouseEvent* event)
{
    m_cursorScreenPos = event->pos();
    m_showCrosshairOverlay = rect().contains(event->pos());

    if (!m_controller.handleMouseRelease(event))
    {
        QOpenGLWidget::mouseReleaseEvent(event);
    }

    updateHoveredWorldPosition(event->pos());
    refreshCommandPrompt();
    update();
} 

// 滚轮缩放：
// 1. 通过 screenToWorld 求出鼠标位置对应的近远平面点
// 2. 构造一条视线
// 3. 与经过 target、法向为 forward 的相机平面求交
// 4. 以该交点为锚点做缩放
void CadViewer::wheelEvent(QWheelEvent* event)
{
    if (!m_controller.handleWheel(event))
    {
        QOpenGLWidget::wheelEvent(event);
    }

    updateHoveredWorldPosition(event->position().toPoint());
}

// 键盘快捷键：
// F / Fit：适配场景
// T / Home：回二维顶视图
// +/-：缩放
void CadViewer::keyPressEvent(QKeyEvent* event)
{
    if (!m_controller.handleKeyPress(event))
    {
        QOpenGLWidget::keyPressEvent(event);
    }

    refreshCommandPrompt();
    update();
}

void CadViewer::dragEnterEvent(QDragEnterEvent* event)
{
    const QMimeData* mimeData = event->mimeData();

    if (mimeData == nullptr || !mimeData->hasUrls())
    {
        event->ignore();
        return;
    }

    for (const QUrl& url : mimeData->urls())
    {
        const QString localFile = url.toLocalFile();

        if (localFile.endsWith(QStringLiteral(".dxf"), Qt::CaseInsensitive)
            || localFile.endsWith(QStringLiteral(".dwg"), Qt::CaseInsensitive))
        {
            event->acceptProposedAction();
            return;
        }
    }

    event->ignore();
}

void CadViewer::dropEvent(QDropEvent* event)
{
    const QMimeData* mimeData = event->mimeData();

    if (mimeData == nullptr || !mimeData->hasUrls())
    {
        event->ignore();
        return;
    }

    for (const QUrl& url : mimeData->urls())
    {
        const QString localFile = url.toLocalFile();

        if (localFile.endsWith(QStringLiteral(".dxf"), Qt::CaseInsensitive)
            || localFile.endsWith(QStringLiteral(".dwg"), Qt::CaseInsensitive))
        {
            emit fileDropRequested(localFile);
            event->acceptProposedAction();
            return;
        }
    }

    event->ignore();
}

// 初始化着色器。
// 当前网格/坐标轴/实体共享同一套简单的 position-color shader 逻辑：
// - 顶点着色器负责 MVP 变换
// - 片元着色器支持圆点裁剪
void CadViewer::initShaders()
{
    static constexpr const char* vertexShaderSource = R"(
        #version 450 core
        layout(location = 0) in vec3 aPosition;
        uniform mat4 uMvp;
        uniform float uPointSize;
        void main()
        {
            gl_Position = uMvp * vec4(aPosition, 1.0);
            gl_PointSize = uPointSize;
        }
    )";

    static constexpr const char* fragmentShaderSource = R"(
        #version 450 core
        uniform vec3 uColor;
        uniform int uRoundPoint;
        out vec4 fragColor;
        void main()
        {
            if (uRoundPoint != 0)
            {
                vec2 pointCoord = gl_PointCoord * 2.0 - vec2(1.0, 1.0);
                if (dot(pointCoord, pointCoord) > 1.0)
                {
                    discard;
                }
            }

            fragColor = vec4(uColor, 1.0);
        }
    )";

    m_entityShader = std::make_unique<QOpenGLShaderProgram>();
    m_entityShader->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    m_entityShader->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    m_entityShader->link();

    m_gridShader = std::make_unique<QOpenGLShaderProgram>();
    m_gridShader->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    m_gridShader->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    m_gridShader->link();
}

// 初始化背景网格顶点缓冲。
// 网格位于 XY 平面，用固定步长和固定范围生成。
void CadViewer::initGridBuffer()
{
    std::vector<QVector3D> vertices;
    constexpr int gridHalfCount = 20;
    constexpr float gridStep = 100.0f;
    constexpr float gridExtent = gridHalfCount * gridStep;

    vertices.reserve((gridHalfCount * 2 + 1) * 4);

    for (int i = -gridHalfCount; i <= gridHalfCount; ++i)
    {
        const float offset = i * gridStep;
        vertices.emplace_back(offset, -gridExtent, 0.0f);
        vertices.emplace_back(offset, gridExtent, 0.0f);
        vertices.emplace_back(-gridExtent, offset, 0.0f);
        vertices.emplace_back(gridExtent, offset, 0.0f);
    }

    m_gridVertexCount = static_cast<int>(vertices.size());

    m_gridVao.create();
    m_gridVao.bind();

    m_gridVbo.create();
    m_gridVbo.bind();
    m_gridVbo.allocate(vertices.data(), m_gridVertexCount * static_cast<int>(sizeof(QVector3D)));

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);

    m_gridVbo.release();
    m_gridVao.release();
}

// 初始化坐标轴缓冲。
// - X/Y 轴固定为实线
// - Z 轴同时预生成实线版本和虚线版本
//   绘制时根据 m_axesSwapped 选择其中一段
void CadViewer::initAxisBuffer()
{
    constexpr float axisLength = 300.0f;
    constexpr float dashLength = 18.0f;
    constexpr float gapLength = 10.0f;

    std::vector<QVector3D> vertices;
    vertices.reserve(64);

    // X 轴。
    vertices.emplace_back(0.0f, 0.0f, 0.0f);
    vertices.emplace_back(axisLength, 0.0f, 0.0f);

    // Y 轴。
    vertices.emplace_back(0.0f, 0.0f, 0.0f);
    vertices.emplace_back(0.0f, axisLength, 0.0f);

    m_axisXyVertexCount = 4;

    // Z 轴实线版本。
    m_axisZSolidOffset = static_cast<int>(vertices.size());
    vertices.emplace_back(0.0f, 0.0f, 0.0f);
    vertices.emplace_back(0.0f, 0.0f, axisLength);
    m_axisZSolidVertexCount = 2;

    // Z 轴虚线版本起始偏移。
    m_axisZDashedOffset = static_cast<int>(vertices.size());

    // 用多段短线生成 Z 轴虚线版本，避免依赖固定管线线型。
    for (float z = 0.0f; z < axisLength; z += dashLength + gapLength)
    {
        const float z0 = z;
        const float z1 = std::min(z + dashLength, axisLength);

        vertices.emplace_back(0.0f, 0.0f, z0);
        vertices.emplace_back(0.0f, 0.0f, z1);
    }

    m_axisZDashedVertexCount = static_cast<int>(vertices.size()) - m_axisZDashedOffset;

    m_axisVao.create();
    m_axisVao.bind();

    m_axisVbo.create();
    m_axisVbo.bind();
    m_axisVbo.allocate(vertices.data(), static_cast<int>(vertices.size() * sizeof(QVector3D)));

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);

    m_axisVbo.release();
    m_axisVao.release();
}

// 初始化轨道中心标记缓冲。
// 当前仅包含一个点，绘制时动态写入 m_orbitCenter。
void CadViewer::initOrbitMarkerBuffer()
{
    const QVector3D initialPoint(0.0f, 0.0f, 0.0f);

    m_orbitMarkerVao.create();
    m_orbitMarkerVao.bind();

    m_orbitMarkerVbo.create();
    m_orbitMarkerVbo.bind();
    m_orbitMarkerVbo.allocate(&initialPoint, static_cast<int>(sizeof(QVector3D)));

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);

    m_orbitMarkerVbo.release();
    m_orbitMarkerVao.release();
}

void CadViewer::initTransientBuffer()
{
    const QVector3D initialPoint(0.0f, 0.0f, 0.0f);

    m_transientVao.create();
    m_transientVao.bind();

    m_transientVbo.create();
    m_transientVbo.bind();
    m_transientVbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    m_transientVbo.allocate(&initialPoint, static_cast<int>(sizeof(QVector3D)));

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);

    m_transientVbo.release();
    m_transientVao.release();
}

// 上传单个实体到 GPU：
// - 为该实体创建 VBO/VAO
// - 记录顶点数、图元类型、颜色
void CadViewer::uploadEntity(const CadItem* entity)
{
    if (entity == nullptr || entity->m_geometry.vertices.isEmpty())
    {
        return;
    }

    const EntityId id = toEntityId(entity);
    removeEntityBuffer(id);

    EntityGpuBuffer& gpuBuffer = m_entityBuffers[id];
    gpuBuffer.vertexCount = entity->m_geometry.vertices.size();
    gpuBuffer.primitiveType = primitiveTypeForEntity(entity);
    gpuBuffer.color = QVector3D(entity->m_color.redF(), entity->m_color.greenF(), entity->m_color.blueF());

    gpuBuffer.vao.create();
    gpuBuffer.vao.bind();

    gpuBuffer.vbo.create();
    gpuBuffer.vbo.bind();
    gpuBuffer.vbo.allocate
    (
        entity->m_geometry.vertices.constData(),
        gpuBuffer.vertexCount * static_cast<int>(sizeof(QVector3D))
    );

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);

    gpuBuffer.vbo.release();
    gpuBuffer.vao.release();
}

// 删除指定实体的 GPU 缓冲。
void CadViewer::removeEntityBuffer(EntityId id)
{
    const auto it = m_entityBuffers.find(id);

    if (it == m_entityBuffers.end())
    {
        return;
    }

    it->second.vao.destroy();
    it->second.vbo.destroy();
    m_entityBuffers.erase(it);
}

// 清空并重建所有实体缓冲。
// 常在切换文档或场景数据变化后调用。
void CadViewer::rebuildAllBuffers()
{
    clearAllBuffers();
    updateSceneBounds();

    if (!m_glInitialized || m_scene == nullptr)
    {
        m_buffersDirty = false;
        return;
    }

    for (auto& entity : m_scene->m_entities)
    {
        uploadEntity(entity.get());
    }

    m_buffersDirty = false;
}

// 释放全部实体 GPU 资源。
void CadViewer::clearAllBuffers()
{
    for (auto& [id, buffer] : m_entityBuffers)
    {
        Q_UNUSED(id);
        buffer.vao.destroy();
        buffer.vbo.destroy();
    }

    m_entityBuffers.clear();
}

// 绘制背景网格。
// 网格不参与深度测试，始终作为背景参考显示。
void CadViewer::renderGrid()
{
    if (m_gridVertexCount <= 0 || !m_gridShader)
    {
        return;
    }

    m_gridShader->bind();
    m_gridShader->setUniformValue("uMvp", m_camera.viewProjectionMatrix(aspectRatio()));
    m_gridShader->setUniformValue("uColor", QVector3D(0.22f, 0.24f, 0.28f));
    m_gridShader->setUniformValue("uPointSize", 1.0f);
    m_gridShader->setUniformValue("uRoundPoint", 0);

    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glLineWidth(1.0f);

    m_gridVao.bind();
    glDrawArrays(GL_LINES, 0, m_gridVertexCount);
    m_gridVao.release();

    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    m_gridShader->release();
}

// 绘制三轴：
// - X 红
// - Y 绿
// - Z 蓝
// 其中 Z 轴可根据 axesSwapped 状态切换实线/虚线。
void CadViewer::renderAxis()
{
    if ((m_axisXyVertexCount <= 0 && m_axisZSolidVertexCount <= 0) || !m_gridShader)
    {
        return;
    }

    m_gridShader->bind();
    m_gridShader->setUniformValue("uMvp", m_camera.viewProjectionMatrix(aspectRatio()));
    m_gridShader->setUniformValue("uPointSize", 1.0f);
    m_gridShader->setUniformValue("uRoundPoint", 0);

    glDisable(GL_DEPTH_TEST);
    glLineWidth(3.0f);

    m_axisVao.bind();

    // X 轴。
    m_gridShader->setUniformValue("uColor", QVector3D(0.95f, 0.30f, 0.25f));
    glDrawArrays(GL_LINES, 0, 2);

    // Y 轴。
    m_gridShader->setUniformValue("uColor", QVector3D(0.25f, 0.85f, 0.35f));
    glDrawArrays(GL_LINES, 2, 2);

    // Z 轴。
    m_gridShader->setUniformValue("uColor", QVector3D(0.30f, 0.55f, 0.95f));
    if (m_camera.axesSwapped())
    {
        glDrawArrays(GL_LINES, m_axisZDashedOffset, m_axisZDashedVertexCount);
    }
    else
    {
        glDrawArrays(GL_LINES, m_axisZSolidOffset, m_axisZSolidVertexCount);
    }

    m_axisVao.release();
    glLineWidth(1.0f);
    glEnable(GL_DEPTH_TEST);
    m_gridShader->release();
}

// 绘制所有实体。
// 当前实体数据默认已是世界坐标，因此直接用 VP 变换。
// 若实体被选中，则使用高亮色和更大的点尺寸。
void CadViewer::renderEntities()
{
    if (m_scene == nullptr || !m_entityShader)
    {
        return;
    }

    m_entityShader->bind();
    m_entityShader->setUniformValue("uMvp", m_camera.viewProjectionMatrix(aspectRatio()));
    m_entityShader->setUniformValue("uRoundPoint", 0);

    for (auto& entity : m_scene->m_entities)
    {
        const EntityId id = toEntityId(entity.get());
        const auto it = m_entityBuffers.find(id);

        if (it == m_entityBuffers.end())
        {
            continue;
        }

        EntityGpuBuffer& buffer = it->second;
        const bool isSelected = id == m_selectedEntityId;
        const QVector3D color = isSelected ? QVector3D(1.0f, 0.80f, 0.15f) : buffer.color;
        const float pointSize = buffer.primitiveType == GL_POINTS ? (isSelected ? 12.0f : 8.0f) : 1.0f;

        m_entityShader->setUniformValue("uColor", color);
        m_entityShader->setUniformValue("uPointSize", pointSize);

        buffer.vao.bind();
        glDrawArrays(buffer.primitiveType, 0, buffer.vertexCount);
        buffer.vao.release();
    }

    m_entityShader->release();
}

// 绘制轨道旋转中心标记。
// 仅在“正在轨道旋转”且场景包围盒有效时显示。
void CadViewer::renderOrbitMarker()
{
    if (m_interactionMode != ViewInteractionMode::Orbiting || !m_hasSceneBounds || !m_entityShader || !m_orbitMarkerVao.isCreated())
    {
        return;
    }

    // 动态更新轨道中心点位置。
    if (m_orbitMarkerVbo.isCreated())
    {
        m_orbitMarkerVbo.bind();
        m_orbitMarkerVbo.write(0, &m_orbitCenter, static_cast<int>(sizeof(QVector3D)));
        m_orbitMarkerVbo.release();
    }

    m_entityShader->bind();
    m_entityShader->setUniformValue("uMvp", m_camera.viewProjectionMatrix(aspectRatio()));
    m_entityShader->setUniformValue("uColor", QVector3D(0.10f, 0.95f, 0.25f));
    m_entityShader->setUniformValue("uPointSize", 14.0f);
    m_entityShader->setUniformValue("uRoundPoint", 1);

    glDisable(GL_DEPTH_TEST);

    m_orbitMarkerVao.bind();
    glDrawArrays(GL_POINTS, 0, 1);
    m_orbitMarkerVao.release();

    glEnable(GL_DEPTH_TEST);
    m_entityShader->release();
}

void CadViewer::renderTransientPrimitives()
{
    if (!m_entityShader || !m_transientVao.isCreated() || !m_transientVbo.isCreated())
    {
        return;
    }

    const std::vector<TransientPrimitive> commandPrimitives = buildTransientPrimitives();
    std::vector<TransientPrimitive> crosshairPrimitives;
    appendCrosshairPrimitives(crosshairPrimitives);

    if (commandPrimitives.empty() && crosshairPrimitives.empty())
    {
        return;
    }

    m_entityShader->bind();
    m_entityShader->setUniformValue("uMvp", m_camera.viewProjectionMatrix(aspectRatio()));
    m_transientVao.bind();

    auto drawPrimitives = [this](const std::vector<TransientPrimitive>& primitives)
    {
        for (const TransientPrimitive& primitive : primitives)
        {
            if (primitive.vertices.isEmpty())
            {
                continue;
            }

            m_transientVbo.bind();
            m_transientVbo.allocate
            (
                primitive.vertices.constData(),
                primitive.vertices.size() * static_cast<int>(sizeof(QVector3D))
            );

            m_entityShader->setUniformValue("uColor", primitive.color);
            m_entityShader->setUniformValue("uPointSize", primitive.pointSize);
            m_entityShader->setUniformValue("uRoundPoint", primitive.roundPoint ? 1 : 0);

            glDrawArrays(primitive.primitiveType, 0, primitive.vertices.size());
            m_transientVbo.release();
        }
    };

    if (!commandPrimitives.empty())
    {
        glDisable(GL_DEPTH_TEST);
        glLineWidth(2.0f);
        drawPrimitives(commandPrimitives);
    }

    if (!crosshairPrimitives.empty())
    {
        glEnable(GL_DEPTH_TEST);
        glLineWidth(1.0f);
        drawPrimitives(crosshairPrimitives);
    }

    m_transientVao.release();
    glLineWidth(1.0f);
    glEnable(GL_DEPTH_TEST);
    m_entityShader->release();
}

// 围绕场景中心做轨道旋转。
// 与 OrbitalCamera::orbit() 的区别是：
// 这里不仅更新 orientation，还会让 eye / target 围绕 pivot 一起旋转，
// 从而实现“绕场景中心看”的效果。
void CadViewer::orbitCameraAroundSceneCenter(float deltaAzimuth, float deltaElevation)
{
    if (!m_hasSceneBounds)
    {
        m_camera.orbit(deltaAzimuth, deltaElevation);
        return;
    }

    const QVector3D pivot = m_orbitCenter;
    QVector3D eye = m_camera.eyePosition();
    QVector3D target = m_camera.target;
    const float distance = m_camera.distance;
    const QQuaternion previousOrientation = normalizedQuaternionOrIdentity(m_camera.orientation);
    QQuaternion candidateOrientation = previousOrientation;
    QVector3D candidateEye = eye;
    QVector3D candidateTarget = target;

    // yaw：围绕世界 Z 轴和 pivot 旋转。
    if (!qFuzzyIsNull(deltaAzimuth))
    {
        const QQuaternion yawRotation =
            QQuaternion::fromAxisAndAngle(QVector3D(0.0f, 0.0f, 1.0f), deltaAzimuth);

        const QVector3D yawedEye = pivot + yawRotation.rotatedVector(candidateEye - pivot);
        const QVector3D yawedTarget = pivot + yawRotation.rotatedVector(candidateTarget - pivot);
        QQuaternion yawedOrientation = yawRotation * candidateOrientation;
        yawedOrientation.normalize();

        if (!violatesViewConstraint(yawedOrientation))
        {
            candidateEye = yawedEye;
            candidateTarget = yawedTarget;
            candidateOrientation = yawedOrientation;
        }
    }

    // pitch：围绕当前相机右轴和 pivot 旋转。
    if (!qFuzzyIsNull(deltaElevation))
    {
        const QVector3D pitchAxis =
            normalizedOr(candidateOrientation.rotatedVector(kLocalRight), kLocalRight);

        const QQuaternion pitchRotation =
            QQuaternion::fromAxisAndAngle(pitchAxis, deltaElevation);

        const QVector3D pitchedEye = pivot + pitchRotation.rotatedVector(candidateEye - pivot);
        const QVector3D pitchedTarget = pivot + pitchRotation.rotatedVector(candidateTarget - pivot);
        QQuaternion pitchedOrientation = pitchRotation * candidateOrientation;
        pitchedOrientation.normalize();

        if (!violatesViewConstraint(pitchedOrientation))
        {
            candidateEye = pitchedEye;
            candidateTarget = pitchedTarget;
            candidateOrientation = pitchedOrientation;
        }
    }

    candidateOrientation.normalize();

    // 当前实现保留 target 围绕 pivot 的更新，distance 仍沿用旧值，
    // 这样 eyePosition() 可继续由 target + orientation + distance 推导。
    m_camera.target = candidateTarget;
    m_camera.distance = distance;
    m_camera.orientation = alignQuaternionHemisphere(previousOrientation, candidateOrientation);
    m_camera.updateAxesSwappedState();
}

// 更新场景包围盒。
// 同时刷新 m_orbitCenter，供 fitScene 和轨道旋转使用。
void CadViewer::updateSceneBounds()
{
    m_hasSceneBounds = false;

    if (m_scene == nullptr || m_scene->m_entities.empty())
    {
        return;
    }

    QVector3D minPoint
    (
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max()
    );

    QVector3D maxPoint
    (
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max()
    );

    for (auto& entity : m_scene->m_entities)
    {
        for (const QVector3D& vertex : entity->m_geometry.vertices)
        {
            minPoint.setX(std::min(minPoint.x(), vertex.x()));
            minPoint.setY(std::min(minPoint.y(), vertex.y()));
            minPoint.setZ(std::min(minPoint.z(), vertex.z()));

            maxPoint.setX(std::max(maxPoint.x(), vertex.x()));
            maxPoint.setY(std::max(maxPoint.y(), vertex.y()));
            maxPoint.setZ(std::max(maxPoint.z(), vertex.z()));
        }
    }

    // 没有有效点则放弃。
    if (minPoint.x() > maxPoint.x())
    {
        return;
    }

    m_sceneMin = minPoint;
    m_sceneMax = maxPoint;
    m_orbitCenter = (m_sceneMin + m_sceneMax) * 0.5f;
    m_hasSceneBounds = true;

    // 若轨道中心标记缓冲已创建，则同步更新。
    if (m_glInitialized && m_orbitMarkerVbo.isCreated())
    {
        m_orbitMarkerVbo.bind();
        m_orbitMarkerVbo.write(0, &m_orbitCenter, static_cast<int>(sizeof(QVector3D)));
        m_orbitMarkerVbo.release();
    }
}

void CadViewer::handleDocumentSceneChanged()
{
    m_buffersDirty = true;
    updateSceneBounds();

    if (findEntityById(m_selectedEntityId) == nullptr)
    {
        m_selectedEntityId = 0;
    }

    if (m_glInitialized)
    {
        makeCurrent();
        rebuildAllBuffers();
        doneCurrent();
    }

    update();
}

void CadViewer::updateHoveredWorldPosition(const QPoint& screenPos)
{
    emit hoveredWorldPositionChanged(screenToGroundPlane(screenPos));
}

QVector3D CadViewer::screenToWorkPlane(const QPoint& screenPos, float planeZ) const
{
    const QVector3D nearPoint = screenToWorld(screenPos, -1.0f);
    const QVector3D farPoint = screenToWorld(screenPos, 1.0f);
    const QVector3D rayDirection = farPoint - nearPoint;

    if (!qFuzzyIsNull(rayDirection.z()))
    {
        const float t = (planeZ - nearPoint.z()) / rayDirection.z();
        return nearPoint + rayDirection * t;
    }

    return QVector3D(nearPoint.x(), nearPoint.y(), planeZ);
}

void CadViewer::appendCrosshairPrimitives(std::vector<TransientPrimitive>& primitives) const
{
    if (!m_showCrosshairOverlay || width() <= 0 || height() <= 0 || m_interactionMode == ViewInteractionMode::Orbiting)
    {
        return;
    }

    const QPoint cursorPoint
    (
        std::clamp(m_cursorScreenPos.x(), 0, std::max(0, width() - 1)),
        std::clamp(m_cursorScreenPos.y(), 0, std::max(0, height() - 1))
    );
    const QVector3D worldCenter = screenToWorkPlane(cursorPoint, m_crosshairPlaneZ);
    const float boxHalfSizeWorld = kPickBoxHalfSizePixels * pixelToWorldScale();

    TransientPrimitive horizontalLine;
    horizontalLine.primitiveType = GL_LINES;
    horizontalLine.color = kCrosshairColor;
    horizontalLine.vertices =
    {
        worldCenter + QVector3D(-kCrosshairHalfLengthWorld, 0.0f, 0.0f),
        worldCenter + QVector3D(kCrosshairHalfLengthWorld, 0.0f, 0.0f)
    };
    primitives.push_back(std::move(horizontalLine));

    TransientPrimitive verticalLine;
    verticalLine.primitiveType = GL_LINES;
    verticalLine.color = kCrosshairColor;
    verticalLine.vertices =
    {
        worldCenter + QVector3D(0.0f, -kCrosshairHalfLengthWorld, 0.0f),
        worldCenter + QVector3D(0.0f, kCrosshairHalfLengthWorld, 0.0f)
    };
    primitives.push_back(std::move(verticalLine));

    TransientPrimitive pickBox;
    pickBox.primitiveType = GL_LINE_STRIP;
    pickBox.color = kCrosshairColor;
    pickBox.vertices =
    {
        worldCenter + QVector3D(-boxHalfSizeWorld, -boxHalfSizeWorld, 0.0f),
        worldCenter + QVector3D(boxHalfSizeWorld, -boxHalfSizeWorld, 0.0f),
        worldCenter + QVector3D(boxHalfSizeWorld, boxHalfSizeWorld, 0.0f),
        worldCenter + QVector3D(-boxHalfSizeWorld, boxHalfSizeWorld, 0.0f),
        worldCenter + QVector3D(-boxHalfSizeWorld, -boxHalfSizeWorld, 0.0f)
    };
    primitives.push_back(std::move(pickBox));
}

// 屏幕空间拾取：
// - 点实体：测鼠标点到投影点距离
// - 线实体：测鼠标点到各投影线段距离
// 返回距离最近且在阈值内的实体 ID。
EntityId CadViewer::pickEntity(const QPoint& screenPos) const
{
    if (m_scene == nullptr)
    {
        return 0;
    }

    const QMatrix4x4 viewProjection = m_camera.viewProjectionMatrix(aspectRatio());
    const QPointF clickPoint(screenPos);
    const float maxDistanceSquared = kPickBoxHalfSizePixels * kPickBoxHalfSizePixels;

    EntityId bestId = 0;
    float bestDistanceSquared = maxDistanceSquared;

    for (auto& entity : m_scene->m_entities)
    {
        const auto& vertices = entity->m_geometry.vertices;

        if (vertices.isEmpty())
        {
            continue;
        }

        float entityDistanceSquared = std::numeric_limits<float>::max();

        if (entity->m_type == DRW::ETYPE::POINT || vertices.size() == 1)
        {
            const QPointF point = projectToScreen(vertices.front(), viewProjection, m_viewportWidth, m_viewportHeight);
            const QPointF delta = clickPoint - point;
            entityDistanceSquared = static_cast<float>(delta.x() * delta.x() + delta.y() * delta.y());
        }
        else
        {
            for (int i = 0; i < vertices.size() - 1; ++i)
            {
                const QPointF start = projectToScreen(vertices.at(i), viewProjection, m_viewportWidth, m_viewportHeight);
                const QPointF end = projectToScreen(vertices.at(i + 1), viewProjection, m_viewportWidth, m_viewportHeight);
                entityDistanceSquared = std::min(entityDistanceSquared, distanceToSegmentSquared(clickPoint, start, end));
            }
        }

        if (entityDistanceSquared <= bestDistanceSquared)
        {
            bestDistanceSquared = entityDistanceSquared;
            bestId = toEntityId(entity.get());
        }
    }

    return bestId;
}

CadItem* CadViewer::findEntityById(EntityId id) const
{
    if (id == 0 || m_scene == nullptr)
    {
        return nullptr;
    }

    for (const std::unique_ptr<CadItem>& entity : m_scene->m_entities)
    {
        if (toEntityId(entity.get()) == id)
        {
            return entity.get();
        }
    }

    return nullptr;
}

std::vector<TransientPrimitive> CadViewer::buildTransientPrimitives() const
{
    std::vector<TransientPrimitive> primitives;
    const DrawStateMachine& state = m_controller.drawState();

    if (!state.hasActiveCommand())
    {
        return primitives;
    }

    if (state.editType == EditType::Move)
    {
        CadItem* selectedItem = selectedEntity();

        if (selectedItem != nullptr && state.moveSubMode == MoveEditSubMode::AwaitTargetPoint && !state.commandPoints.isEmpty())
        {
            const QVector3D basePoint = state.commandPoints.front();
            const QVector3D delta = state.currentPos - basePoint;

            TransientPrimitive movedEntityPreview;
            movedEntityPreview.primitiveType = primitiveTypeForEntity(selectedItem);
            movedEntityPreview.color = QVector3D(0.98f, 0.67f, 0.12f);
            movedEntityPreview.pointSize = movedEntityPreview.primitiveType == GL_POINTS ? 12.0f : 1.0f;
            movedEntityPreview.roundPoint = movedEntityPreview.primitiveType == GL_POINTS;

            movedEntityPreview.vertices.reserve(selectedItem->m_geometry.vertices.size());

            for (const QVector3D& vertex : selectedItem->m_geometry.vertices)
            {
                movedEntityPreview.vertices.append(vertex + delta);
            }

            primitives.push_back(std::move(movedEntityPreview));

            TransientPrimitive guideLine;
            guideLine.primitiveType = GL_LINES;
            guideLine.color = QVector3D(0.35f, 0.90f, 1.0f);
            guideLine.vertices = { basePoint, state.currentPos };
            primitives.push_back(std::move(guideLine));
        }

        return primitives;
    }

    switch (state.drawType)
    {
    case DrawType::Point:
    {
        if (state.pointSubMode == PointDrawSubMode::AwaitPosition)
        {
            TransientPrimitive pointPreview;
            pointPreview.primitiveType = GL_POINTS;
            pointPreview.color = QVector3D(0.35f, 0.90f, 1.0f);
            pointPreview.pointSize = 10.0f;
            pointPreview.roundPoint = true;
            pointPreview.vertices = { flattenedToGroundPlane(state.currentPos) };
            primitives.push_back(std::move(pointPreview));
        }
        break;
    }
    case DrawType::Line:
    {
        if (state.lineSubMode == LineDrawSubMode::AwaitEndPoint && !state.commandPoints.isEmpty())
        {
            TransientPrimitive linePreview;
            linePreview.primitiveType = GL_LINES;
            linePreview.color = QVector3D(0.35f, 0.90f, 1.0f);
            linePreview.vertices = { state.commandPoints.front(), flattenedToGroundPlane(state.currentPos) };
            primitives.push_back(std::move(linePreview));
        }
        break;
    }
    case DrawType::Circle:
    {
        if (state.circleSubMode == CircleDrawSubMode::AwaitRadius && !state.commandPoints.isEmpty())
        {
            const QVector3D center = state.commandPoints.front();
            const QVector3D currentPoint = flattenedToGroundPlane(state.currentPos);
            const float radius = (currentPoint - center).length();

            TransientPrimitive radiusLine;
            radiusLine.primitiveType = GL_LINES;
            radiusLine.color = QVector3D(0.35f, 0.90f, 1.0f);
            radiusLine.vertices = { center, currentPoint };
            primitives.push_back(std::move(radiusLine));

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
        if (state.arcSubMode == ArcDrawSubMode::AwaitRadius && !state.commandPoints.isEmpty())
        {
            TransientPrimitive radiusLine;
            radiusLine.primitiveType = GL_LINES;
            radiusLine.color = QVector3D(0.35f, 0.90f, 1.0f);
            radiusLine.vertices = { state.commandPoints.front(), flattenedToGroundPlane(state.currentPos) };
            primitives.push_back(std::move(radiusLine));
        }
        else if (state.arcSubMode == ArcDrawSubMode::AwaitStartAngle && state.commandPoints.size() >= 2)
        {
            const QVector3D center = state.commandPoints[0];
            const QVector3D radiusPoint = state.commandPoints[1];
            const QVector3D startPoint = projectPointToRadius(center, radiusPoint, flattenedToGroundPlane(state.currentPos));

            TransientPrimitive circlePreview;
            circlePreview.primitiveType = GL_LINE_STRIP;
            circlePreview.color = QVector3D(0.35f, 0.90f, 1.0f);
            appendCirclePreview(circlePreview.vertices, center, (radiusPoint - center).length());
            primitives.push_back(std::move(circlePreview));

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
            const QVector3D endPoint = projectPointToRadius(center, radiusPoint, flattenedToGroundPlane(state.currentPos));

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
            TransientPrimitive axisLine;
            axisLine.primitiveType = GL_LINES;
            axisLine.color = QVector3D(0.35f, 0.90f, 1.0f);
            axisLine.vertices = { state.commandPoints.front(), flattenedToGroundPlane(state.currentPos) };
            primitives.push_back(std::move(axisLine));
        }
        else if (state.ellipseSubMode == EllipseDrawSubMode::AwaitMinorAxis && state.commandPoints.size() >= 2)
        {
            const QVector3D center = state.commandPoints[0];
            const QVector3D axisEnd = state.commandPoints[1];
            const QVector3D currentPoint = flattenedToGroundPlane(state.currentPos);

            TransientPrimitive axisLine;
            axisLine.primitiveType = GL_LINES;
            axisLine.color = QVector3D(0.35f, 0.90f, 1.0f);
            axisLine.vertices = { center, axisEnd };
            primitives.push_back(std::move(axisLine));

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

            polylinePreview.vertices.append(flattenedToGroundPlane(state.commandPoints.front()));

            for (int i = 0; i + 1 < state.commandPoints.size(); ++i)
            {
                const QVector3D segmentStart = flattenedToGroundPlane(state.commandPoints[i]);
                const QVector3D segmentEnd = flattenedToGroundPlane(state.commandPoints[i + 1]);
                const double bulge = i < state.commandBulges.size() ? state.commandBulges[i] : 0.0;
                appendBulgePreview(polylinePreview.vertices, segmentStart, segmentEnd, bulge);
            }

            const QVector3D currentPoint = flattenedToGroundPlane(state.currentPos);
            const QVector3D lastPoint = flattenedToGroundPlane(state.commandPoints.back());

            if ((currentPoint - lastPoint).lengthSquared() > 1.0e-10f)
            {
                const bool arcMode = state.drawType == DrawType::Polyline
                    ? state.polylineSubMode == PolylineDrawSubMode::AwaitArcEndPoint
                    : state.lwPolylineSubMode == LWPolylineDrawSubMode::AwaitArcEndPoint;

                if (arcMode && state.commandPoints.size() >= 2)
                {
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

// 计算当前视口宽高比。
float CadViewer::aspectRatio() const
{
    return static_cast<float>(m_viewportWidth) / static_cast<float>(std::max(1, m_viewportHeight));
}

// 将 1 个屏幕像素估算为多少世界单位。
// 当前基于正交投影 viewHeight 直接换算，适用于平移操作。
float CadViewer::pixelToWorldScale() const
{
    return m_camera.viewHeight / static_cast<float>(std::max(1, m_viewportHeight));
}
