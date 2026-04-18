#pragma once

#include <QVector2D>

class CrossSection2D
{
public:
    virtual ~CrossSection2D() = default;

    virtual QVector2D pointAt(double u) const = 0;
    virtual QVector2D tangentAt(double u) const = 0;
    virtual QVector2D outwardNormalAt(double u) const = 0;
};

class RoundedRectSection2D : public CrossSection2D
{
public:
    RoundedRectSection2D
    (
        double halfWidthY,
        double halfHeightZ,
        double cornerRadiusY,
        double cornerRadiusZ
    );

    QVector2D pointAt(double u) const override;
    QVector2D tangentAt(double u) const override;
    QVector2D outwardNormalAt(double u) const override;

    bool isValid() const;

private:
    struct SegmentSample
    {
        QVector2D point;
        QVector2D tangent;
        QVector2D normal;
    };

    SegmentSample sampleByArcLength(double arcLength) const;
    static QVector2D normalizedOrFallback(const QVector2D& value, const QVector2D& fallback);

private:
    double m_halfWidthY = 0.0;
    double m_halfHeightZ = 0.0;
    double m_cornerRadiusY = 0.0;
    double m_cornerRadiusZ = 0.0;
    double m_straightLengthY = 0.0;
    double m_straightLengthZ = 0.0;
    double m_cornerQuarterLength = 0.0;
    double m_totalLength = 0.0;
};

