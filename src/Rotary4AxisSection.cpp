#include "pch.h"

#include "Rotary4AxisSection.h"

#include <algorithm>
#include <cmath>

namespace
{
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kEpsilon = 1.0e-9;

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
        return sample;
    }

    if (arcLength < leftEdge || m_cornerQuarterLength <= kEpsilon)
    {
        const double t = m_straightLengthZ > kEpsilon ? clamp01((arcLength - topLeftCorner) / m_straightLengthZ) : 0.0;
        const double z = innerZ - 2.0 * innerZ * t;
        sample.point = QVector2D(static_cast<float>(-m_halfWidthY), static_cast<float>(z));
        sample.tangent = QVector2D(0.0f, -1.0f);
        sample.normal = QVector2D(-1.0f, 0.0f);
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
        return sample;
    }

    if (arcLength < bottomEdge || m_cornerQuarterLength <= kEpsilon)
    {
        const double t = m_straightLengthY > kEpsilon ? clamp01((arcLength - bottomLeftCorner) / m_straightLengthY) : 0.0;
        const double y = -innerY + 2.0 * innerY * t;
        sample.point = QVector2D(static_cast<float>(y), static_cast<float>(-m_halfHeightZ));
        sample.tangent = QVector2D(1.0f, 0.0f);
        sample.normal = QVector2D(0.0f, -1.0f);
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
        return sample;
    }

    if (arcLength < rightEdge || m_cornerQuarterLength <= kEpsilon)
    {
        const double t = m_straightLengthZ > kEpsilon ? clamp01((arcLength - bottomRightCorner) / m_straightLengthZ) : 0.0;
        const double z = -innerZ + 2.0 * innerZ * t;
        sample.point = QVector2D(static_cast<float>(m_halfWidthY), static_cast<float>(z));
        sample.tangent = QVector2D(0.0f, 1.0f);
        sample.normal = QVector2D(1.0f, 0.0f);
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

