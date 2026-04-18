#include "pch.h"

#include "Rotary4AxisSection.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kEpsilon = 1.0e-9;
    constexpr double kProjectionBias = 1.0e-6;

    double clamp01(double value)
    {
        return std::max(0.0, std::min(1.0, value));
    }

    double normalizeUnit(double u)
    {
        if (u >= 1.0 || u < 0.0)
        {
            u = std::fmod(u, 1.0);

            if (u < 0.0)
            {
                u += 1.0;
            }
        }

        return u;
    }

    double quarterEllipseLength(double radiusY, double radiusZ)
    {
        if (radiusY <= kEpsilon || radiusZ <= kEpsilon)
        {
            return 0.0;
        }

        const double a = std::max(radiusY, radiusZ);
        const double b = std::min(radiusY, radiusZ);
        const double perimeter = kPi * (3.0 * (a + b) - std::sqrt((3.0 * a + b) * (a + 3.0 * b)));
        return perimeter * 0.25;
    }

    double squaredLength(const QVector2D& value)
    {
        return static_cast<double>(value.lengthSquared());
    }
}

SectionRegionType CrossSection2D::regionTypeAt(double u) const
{
    Q_UNUSED(u);
    return SectionRegionType::Unknown;
}

int CrossSection2D::sideIndexAt(double u) const
{
    Q_UNUSED(u);
    return -1;
}

RoundedRectSection2D::RoundedRectSection2D
(
    double halfWidthY,
    double halfHeightZ,
    double cornerRadiusY,
    double cornerRadiusZ
)
    : m_halfWidthY(std::max(0.0, halfWidthY))
    , m_halfHeightZ(std::max(0.0, halfHeightZ))
    , m_cornerRadiusY(std::max(0.0, std::min(cornerRadiusY, m_halfWidthY)))
    , m_cornerRadiusZ(std::max(0.0, std::min(cornerRadiusZ, m_halfHeightZ)))
{
    m_straightLengthY = std::max(0.0, (m_halfWidthY - m_cornerRadiusY) * 2.0);
    m_straightLengthZ = std::max(0.0, (m_halfHeightZ - m_cornerRadiusZ) * 2.0);
    m_cornerQuarterLength = quarterEllipseLength(m_cornerRadiusY, m_cornerRadiusZ);
    m_totalLength = 2.0 * (m_straightLengthY + m_straightLengthZ) + 4.0 * m_cornerQuarterLength;
}

bool RoundedRectSection2D::isValid() const
{
    return m_halfWidthY > kEpsilon
        && m_halfHeightZ > kEpsilon
        && m_totalLength > kEpsilon;
}

QVector2D RoundedRectSection2D::normalizedOrFallback(const QVector2D& value, const QVector2D& fallback)
{
    const float lengthSquared = value.lengthSquared();

    if (lengthSquared <= static_cast<float>(kEpsilon))
    {
        return fallback;
    }

    QVector2D normalized = value;
    normalized.normalize();
    return normalized;
}

RoundedRectSection2D::SegmentSample RoundedRectSection2D::sampleByArcLength(double arcLength) const
{
    SegmentSample sample;
    sample.point = QVector2D();
    sample.tangent = QVector2D(1.0f, 0.0f);
    sample.normal = QVector2D(0.0f, 1.0f);
    sample.regionType = SectionRegionType::Unknown;
    sample.sideIndex = -1;

    if (!isValid())
    {
        return sample;
    }

    arcLength = std::fmod(std::max(0.0, arcLength), m_totalLength);
    const double innerY = std::max(0.0, m_halfWidthY - m_cornerRadiusY);
    const double innerZ = std::max(0.0, m_halfHeightZ - m_cornerRadiusZ);

    const double topEdge = m_straightLengthY;
    const double topLeftCorner = topEdge + m_cornerQuarterLength;
    const double leftEdge = topLeftCorner + m_straightLengthZ;
    const double bottomLeftCorner = leftEdge + m_cornerQuarterLength;
    const double bottomEdge = bottomLeftCorner + m_straightLengthY;
    const double bottomRightCorner = bottomEdge + m_cornerQuarterLength;
    const double rightEdge = bottomRightCorner + m_straightLengthZ;

    if (arcLength < topEdge || m_cornerQuarterLength <= kEpsilon)
    {
        const double t = topEdge > kEpsilon ? clamp01(arcLength / topEdge) : 0.0;
        const double y = innerY - 2.0 * innerY * t;
        sample.point = QVector2D(static_cast<float>(y), static_cast<float>(m_halfHeightZ));
        sample.tangent = QVector2D(-1.0f, 0.0f);
        sample.normal = QVector2D(0.0f, 1.0f);
        sample.regionType = SectionRegionType::FlatSide;
        sample.sideIndex = 0;
        return sample;
    }

    if (arcLength < topLeftCorner)
    {
        const double t = clamp01((arcLength - topEdge) / m_cornerQuarterLength);
        const double theta = kPi * 0.5 + t * (kPi * 0.5);
        const double cy = -innerY;
        const double cz = innerZ;
        const double y = cy + m_cornerRadiusY * std::cos(theta);
        const double z = cz + m_cornerRadiusZ * std::sin(theta);
        const QVector2D tangent
        (
            static_cast<float>(-m_cornerRadiusY * std::sin(theta)),
            static_cast<float>(m_cornerRadiusZ * std::cos(theta))
        );
        const QVector2D normal
        (
            static_cast<float>((y - cy) / std::max(kEpsilon, m_cornerRadiusY * m_cornerRadiusY)),
            static_cast<float>((z - cz) / std::max(kEpsilon, m_cornerRadiusZ * m_cornerRadiusZ))
        );

        sample.point = QVector2D(static_cast<float>(y), static_cast<float>(z));
        sample.tangent = normalizedOrFallback(tangent, QVector2D(0.0f, -1.0f));
        sample.normal = normalizedOrFallback(normal, QVector2D(-1.0f, 0.0f));
        sample.regionType = SectionRegionType::CornerTransition;
        return sample;
    }

    if (arcLength < leftEdge || m_cornerQuarterLength <= kEpsilon)
    {
        const double t = m_straightLengthZ > kEpsilon ? clamp01((arcLength - topLeftCorner) / m_straightLengthZ) : 0.0;
        const double z = innerZ - 2.0 * innerZ * t;
        sample.point = QVector2D(static_cast<float>(-m_halfWidthY), static_cast<float>(z));
        sample.tangent = QVector2D(0.0f, -1.0f);
        sample.normal = QVector2D(-1.0f, 0.0f);
        sample.regionType = SectionRegionType::FlatSide;
        sample.sideIndex = 1;
        return sample;
    }

    if (arcLength < bottomLeftCorner)
    {
        const double t = clamp01((arcLength - leftEdge) / m_cornerQuarterLength);
        const double theta = kPi + t * (kPi * 0.5);
        const double cy = -innerY;
        const double cz = -innerZ;
        const double y = cy + m_cornerRadiusY * std::cos(theta);
        const double z = cz + m_cornerRadiusZ * std::sin(theta);
        const QVector2D tangent
        (
            static_cast<float>(-m_cornerRadiusY * std::sin(theta)),
            static_cast<float>(m_cornerRadiusZ * std::cos(theta))
        );
        const QVector2D normal
        (
            static_cast<float>((y - cy) / std::max(kEpsilon, m_cornerRadiusY * m_cornerRadiusY)),
            static_cast<float>((z - cz) / std::max(kEpsilon, m_cornerRadiusZ * m_cornerRadiusZ))
        );

        sample.point = QVector2D(static_cast<float>(y), static_cast<float>(z));
        sample.tangent = normalizedOrFallback(tangent, QVector2D(1.0f, 0.0f));
        sample.normal = normalizedOrFallback(normal, QVector2D(0.0f, -1.0f));
        sample.regionType = SectionRegionType::CornerTransition;
        return sample;
    }

    if (arcLength < bottomEdge || m_cornerQuarterLength <= kEpsilon)
    {
        const double t = m_straightLengthY > kEpsilon ? clamp01((arcLength - bottomLeftCorner) / m_straightLengthY) : 0.0;
        const double y = -innerY + 2.0 * innerY * t;
        sample.point = QVector2D(static_cast<float>(y), static_cast<float>(-m_halfHeightZ));
        sample.tangent = QVector2D(1.0f, 0.0f);
        sample.normal = QVector2D(0.0f, -1.0f);
        sample.regionType = SectionRegionType::FlatSide;
        sample.sideIndex = 2;
        return sample;
    }

    if (arcLength < bottomRightCorner)
    {
        const double t = clamp01((arcLength - bottomEdge) / m_cornerQuarterLength);
        const double theta = kPi * 1.5 + t * (kPi * 0.5);
        const double cy = innerY;
        const double cz = -innerZ;
        const double y = cy + m_cornerRadiusY * std::cos(theta);
        const double z = cz + m_cornerRadiusZ * std::sin(theta);
        const QVector2D tangent
        (
            static_cast<float>(-m_cornerRadiusY * std::sin(theta)),
            static_cast<float>(m_cornerRadiusZ * std::cos(theta))
        );
        const QVector2D normal
        (
            static_cast<float>((y - cy) / std::max(kEpsilon, m_cornerRadiusY * m_cornerRadiusY)),
            static_cast<float>((z - cz) / std::max(kEpsilon, m_cornerRadiusZ * m_cornerRadiusZ))
        );

        sample.point = QVector2D(static_cast<float>(y), static_cast<float>(z));
        sample.tangent = normalizedOrFallback(tangent, QVector2D(0.0f, 1.0f));
        sample.normal = normalizedOrFallback(normal, QVector2D(1.0f, 0.0f));
        sample.regionType = SectionRegionType::CornerTransition;
        return sample;
    }

    if (arcLength < rightEdge || m_cornerQuarterLength <= kEpsilon)
    {
        const double t = m_straightLengthZ > kEpsilon ? clamp01((arcLength - bottomRightCorner) / m_straightLengthZ) : 0.0;
        const double z = -innerZ + 2.0 * innerZ * t;
        sample.point = QVector2D(static_cast<float>(m_halfWidthY), static_cast<float>(z));
        sample.tangent = QVector2D(0.0f, 1.0f);
        sample.normal = QVector2D(1.0f, 0.0f);
        sample.regionType = SectionRegionType::FlatSide;
        sample.sideIndex = 3;
        return sample;
    }

    const double t = m_cornerQuarterLength > kEpsilon ? clamp01((arcLength - rightEdge) / m_cornerQuarterLength) : 1.0;
    const double theta = t * (kPi * 0.5);
    const double cy = innerY;
    const double cz = innerZ;
    const double y = cy + m_cornerRadiusY * std::cos(theta);
    const double z = cz + m_cornerRadiusZ * std::sin(theta);
    const QVector2D tangent
    (
        static_cast<float>(-m_cornerRadiusY * std::sin(theta)),
        static_cast<float>(m_cornerRadiusZ * std::cos(theta))
    );
    const QVector2D normal
    (
        static_cast<float>((y - cy) / std::max(kEpsilon, m_cornerRadiusY * m_cornerRadiusY)),
        static_cast<float>((z - cz) / std::max(kEpsilon, m_cornerRadiusZ * m_cornerRadiusZ))
    );

    sample.point = QVector2D(static_cast<float>(y), static_cast<float>(z));
    sample.tangent = normalizedOrFallback(tangent, QVector2D(-1.0f, 0.0f));
    sample.normal = normalizedOrFallback(normal, QVector2D(0.0f, 1.0f));
    sample.regionType = SectionRegionType::CornerTransition;
    return sample;
}

QVector2D RoundedRectSection2D::pointAt(double u) const
{
    if (!isValid())
    {
        return QVector2D();
    }

    const double normalized = normalizeUnit(u);
    const double arcLength = normalized * m_totalLength;
    return sampleByArcLength(arcLength).point;
}

QVector2D RoundedRectSection2D::tangentAt(double u) const
{
    if (!isValid())
    {
        return QVector2D(1.0f, 0.0f);
    }

    const double normalized = normalizeUnit(u);
    const double arcLength = normalized * m_totalLength;
    return sampleByArcLength(arcLength).tangent;
}

QVector2D RoundedRectSection2D::outwardNormalAt(double u) const
{
    if (!isValid())
    {
        return QVector2D(0.0f, 1.0f);
    }

    const double normalized = normalizeUnit(u);
    const double arcLength = normalized * m_totalLength;
    return sampleByArcLength(arcLength).normal;
}

SectionRegionType RoundedRectSection2D::regionTypeAt(double u) const
{
    if (!isValid())
    {
        return SectionRegionType::Unknown;
    }

    const double normalized = normalizeUnit(u);
    const double arcLength = normalized * m_totalLength;
    return sampleByArcLength(arcLength).regionType;
}

int RoundedRectSection2D::sideIndexAt(double u) const
{
    if (!isValid())
    {
        return -1;
    }

    const double normalized = normalizeUnit(u);
    const double arcLength = normalized * m_totalLength;
    return sampleByArcLength(arcLength).sideIndex;
}

QVector2D RoundedRectSection2D::flatSideNormal(int sideIndex) const
{
    switch (sideIndex)
    {
    case 0:
        return QVector2D(0.0f, 1.0f);
    case 1:
        return QVector2D(-1.0f, 0.0f);
    case 2:
        return QVector2D(0.0f, -1.0f);
    case 3:
        return QVector2D(1.0f, 0.0f);
    default:
        return QVector2D(0.0f, 0.0f);
    }
}

bool RoundedRectSection2D::projectPoint(const QVector2D& yzPoint, ProjectionResult& result) const
{
    result = ProjectionResult();

    if (!isValid())
    {
        return false;
    }

    const double innerY = std::max(0.0, m_halfWidthY - m_cornerRadiusY);
    const double innerZ = std::max(0.0, m_halfHeightZ - m_cornerRadiusZ);
    const double y = yzPoint.x();
    const double z = yzPoint.y();
    const double absY = std::abs(y);
    const double absZ = std::abs(z);
    const double sideGateTol = 1.0e-5;

    auto assignFlatSide = [&](int sideIndex, double clampedY, double clampedZ)
    {
        result.point = QVector2D(static_cast<float>(clampedY), static_cast<float>(clampedZ));
        result.normal = flatSideNormal(sideIndex);
        result.regionType = SectionRegionType::FlatSide;
        result.sideIndex = sideIndex;
    };

    double bestSideDistance = std::numeric_limits<double>::max();
    int bestSideIndex = -1;

    if (absY <= innerY + sideGateTol)
    {
        const double topDistance = std::abs(z - m_halfHeightZ);
        const double bottomDistance = std::abs(z + m_halfHeightZ);

        if (topDistance < bestSideDistance)
        {
            bestSideDistance = topDistance;
            bestSideIndex = 0;
        }

        if (bottomDistance < bestSideDistance)
        {
            bestSideDistance = bottomDistance;
            bestSideIndex = 2;
        }
    }

    if (absZ <= innerZ + sideGateTol)
    {
        const double leftDistance = std::abs(y + m_halfWidthY);
        const double rightDistance = std::abs(y - m_halfWidthY);

        if (leftDistance < bestSideDistance)
        {
            bestSideDistance = leftDistance;
            bestSideIndex = 1;
        }

        if (rightDistance < bestSideDistance)
        {
            bestSideDistance = rightDistance;
            bestSideIndex = 3;
        }
    }

    RoundedRectSection2D::ProjectionResult bestCorner;
    double bestCornerDistance = std::numeric_limits<double>::max();

    const auto evaluateCorner =
        [&](double centerY, double centerZ, double angleStart, double angleEnd)
    {
        if (m_cornerRadiusY <= kEpsilon || m_cornerRadiusZ <= kEpsilon)
        {
            return;
        }

        const double scaledY = (y - centerY) / std::max(kEpsilon, m_cornerRadiusY);
        const double scaledZ = (z - centerZ) / std::max(kEpsilon, m_cornerRadiusZ);
        double theta = std::atan2(scaledZ, scaledY);

        while (theta < angleStart)
        {
            theta += 2.0 * kPi;
        }

        while (theta > angleEnd)
        {
            theta -= 2.0 * kPi;
        }

        theta = std::max(angleStart, std::min(angleEnd, theta));

        ProjectionResult candidate;
        candidate.point = QVector2D
        (
            static_cast<float>(centerY + m_cornerRadiusY * std::cos(theta)),
            static_cast<float>(centerZ + m_cornerRadiusZ * std::sin(theta))
        );
        const QVector2D normal
        (
            static_cast<float>((candidate.point.x() - centerY) / std::max(kEpsilon, m_cornerRadiusY * m_cornerRadiusY)),
            static_cast<float>((candidate.point.y() - centerZ) / std::max(kEpsilon, m_cornerRadiusZ * m_cornerRadiusZ))
        );
        candidate.normal = normalizedOrFallback(normal, QVector2D(0.0f, 1.0f));
        candidate.regionType = SectionRegionType::CornerTransition;
        candidate.sideIndex = -1;

        const double distance = squaredLength(yzPoint - candidate.point);

        if (distance < bestCornerDistance)
        {
            bestCornerDistance = distance;
            bestCorner = candidate;
        }
    };

    evaluateCorner(-innerY, innerZ, kPi * 0.5, kPi);
    evaluateCorner(-innerY, -innerZ, kPi, kPi * 1.5);
    evaluateCorner(innerY, -innerZ, kPi * 1.5, kPi * 2.0);
    evaluateCorner(innerY, innerZ, 0.0, kPi * 0.5);

    if (bestSideIndex >= 0 && (bestSideDistance * bestSideDistance) <= bestCornerDistance + kProjectionBias)
    {
        switch (bestSideIndex)
        {
        case 0:
            assignFlatSide(0, std::max(-innerY, std::min(innerY, y)), m_halfHeightZ);
            return true;
        case 1:
            assignFlatSide(1, -m_halfWidthY, std::max(-innerZ, std::min(innerZ, z)));
            return true;
        case 2:
            assignFlatSide(2, std::max(-innerY, std::min(innerY, y)), -m_halfHeightZ);
            return true;
        case 3:
            assignFlatSide(3, m_halfWidthY, std::max(-innerZ, std::min(innerZ, z)));
            return true;
        default:
            break;
        }
    }

    result = bestCorner;
    return result.regionType != SectionRegionType::Unknown;
}
