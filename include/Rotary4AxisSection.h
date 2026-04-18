#pragma once

#include "Rotary4AxisTypes.h"

#include <QVector2D>

class CrossSection2D
{
public:
    virtual ~CrossSection2D() = default;

    virtual QVector2D pointAt(double u) const = 0;
    virtual QVector2D tangentAt(double u) const = 0;
    virtual QVector2D outwardNormalAt(double u) const = 0;
    virtual SectionRegionType regionTypeAt(double u) const;
    virtual int sideIndexAt(double u) const;
};

class RoundedRectSection2D : public CrossSection2D
{
public:
    struct ProjectionResult
    {
        QVector2D point;
        QVector2D normal;
        SectionRegionType regionType = SectionRegionType::Unknown;
        int sideIndex = -1;
    };

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
    SectionRegionType regionTypeAt(double u) const override;
    int sideIndexAt(double u) const override;
    bool projectPoint(const QVector2D& yzPoint, ProjectionResult& result) const;
    QVector2D flatSideNormal(int sideIndex) const;

    bool isValid() const;

private:
    struct SegmentSample
    {
        QVector2D point;
        QVector2D tangent;
        QVector2D normal;
        SectionRegionType regionType = SectionRegionType::Unknown;
        int sideIndex = -1;
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
