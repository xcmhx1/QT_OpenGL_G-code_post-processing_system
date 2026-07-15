// 右上角三维视图方块：跟随相机方向显示，并提供标准视角切换。
#include "platform/pch.h"

#include "cad/view/CadViewer.h"

#include <QPainter>

#include <algorithm>
#include <array>

namespace
{
    constexpr float kCubeCameraDistance = 4.2f;
    constexpr float kCubeScale = 28.0f;
    constexpr int kCubePanelSize = 116;
    constexpr int kCubePanelMargin = 14;
    constexpr int kNavigationStrip = 20;

    struct FaceDefinition
    {
        int faceId;
        QVector3D normal;
        std::array<QVector3D, 4> vertices;
        const char* label;
    };

    const std::array<FaceDefinition, 6> kFaces =
    {{
        { 1, QVector3D(0.0f, 0.0f, 1.0f),
            { QVector3D(-1, -1, 1), QVector3D(1, -1, 1), QVector3D(1, 1, 1), QVector3D(-1, 1, 1) }, "上" },
        { 2, QVector3D(0.0f, 0.0f, -1.0f),
            { QVector3D(-1, 1, -1), QVector3D(1, 1, -1), QVector3D(1, -1, -1), QVector3D(-1, -1, -1) }, "下" },
        { 3, QVector3D(0.0f, -1.0f, 0.0f),
            { QVector3D(-1, -1, -1), QVector3D(1, -1, -1), QVector3D(1, -1, 1), QVector3D(-1, -1, 1) }, "前" },
        { 4, QVector3D(0.0f, 1.0f, 0.0f),
            { QVector3D(1, 1, -1), QVector3D(-1, 1, -1), QVector3D(-1, 1, 1), QVector3D(1, 1, 1) }, "后" },
        { 5, QVector3D(-1.0f, 0.0f, 0.0f),
            { QVector3D(-1, 1, -1), QVector3D(-1, -1, -1), QVector3D(-1, -1, 1), QVector3D(-1, 1, 1) }, "左" },
        { 6, QVector3D(1.0f, 0.0f, 0.0f),
            { QVector3D(1, -1, -1), QVector3D(1, 1, -1), QVector3D(1, 1, 1), QVector3D(1, -1, 1) }, "右" }
    }};

    QColor blended(const QColor& first, const QColor& second, int secondWeight)
    {
        const int firstWeight = 100 - secondWeight;
        return QColor
        (
            (first.red() * firstWeight + second.red() * secondWeight) / 100,
            (first.green() * firstWeight + second.green() * secondWeight) / 100,
            (first.blue() * firstWeight + second.blue() * secondWeight) / 100
        );
    }
}

QVector<CadViewer::ViewCubeFaceOverlay> CadViewer::buildViewCubeFaces() const
{
    QVector<ViewCubeFaceOverlay> overlays;

    if (width() < kCubePanelSize + kCubePanelMargin * 2
        || height() < kCubePanelSize + kCubePanelMargin * 2)
    {
        return overlays;
    }

    const QPointF center
    (
        width() - kCubePanelMargin - kCubePanelSize * 0.5,
        kCubePanelMargin + kCubePanelSize * 0.5
    );
    const QVector3D cameraRight = m_camera.rightDirection();
    const QVector3D cameraUp = m_camera.upDirection();
    const QVector3D cameraForward = m_camera.forwardDirection();

    for (const FaceDefinition& definition : kFaces)
    {
        if (QVector3D::dotProduct(definition.normal, cameraForward) >= -1.0e-4f)
        {
            continue;
        }

        ViewCubeFaceOverlay overlay;
        overlay.face = static_cast<ViewCubeFace>(definition.faceId);
        overlay.label = QString::fromUtf8(definition.label);
        QVector3D faceCenter;

        for (const QVector3D& vertex : definition.vertices)
        {
            const float localX = QVector3D::dotProduct(vertex, cameraRight);
            const float localY = QVector3D::dotProduct(vertex, cameraUp);
            const float localDepth = QVector3D::dotProduct(vertex, cameraForward);
            const float perspective = kCubeCameraDistance / std::max(1.0f, kCubeCameraDistance + localDepth);
            overlay.polygon.append
            (
                center + QPointF(localX * perspective * kCubeScale, -localY * perspective * kCubeScale)
            );
            faceCenter += vertex;
        }

        faceCenter /= static_cast<float>(definition.vertices.size());
        overlay.depth = QVector3D::dotProduct(faceCenter, cameraForward);
        overlays.push_back(std::move(overlay));
    }

    std::sort(overlays.begin(), overlays.end(), [](const ViewCubeFaceOverlay& left, const ViewCubeFaceOverlay& right)
    {
        return left.depth > right.depth;
    });
    return overlays;
}

void CadViewer::renderViewCube()
{
    const QVector<ViewCubeFaceOverlay> faces = buildViewCubeFaces();

    if (faces.isEmpty())
    {
        return;
    }

    const QRect panelRect
    (
        width() - kCubePanelMargin - kCubePanelSize,
        kCubePanelMargin,
        kCubePanelSize,
        kCubePanelSize
    );
    QColor panelColor = m_theme.panelBackground;
    panelColor.setAlpha(m_theme.dark ? 218 : 232);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QPen(m_theme.borderColor, 1.0));
    painter.setBrush(panelColor);
    painter.drawRoundedRect(panelRect, 8.0, 8.0);

    QFont labelFont = painter.font();
    labelFont.setPointSize(9);
    labelFont.setBold(true);
    painter.setFont(labelFont);

    for (const ViewCubeFaceOverlay& face : faces)
    {
        QColor fill = face.face == ViewCubeFace::Top
            ? blended(m_theme.surfaceBackground, m_theme.accentColor, 10)
            : blended(m_theme.surfaceAltBackground, m_theme.surfaceBackground, 42);

        if (face.face == m_hoveredViewCubeFace)
        {
            fill = blended(m_theme.accentColor, m_theme.surfaceBackground, 18);
        }

        painter.setPen(QPen(face.face == m_hoveredViewCubeFace ? m_theme.accentColor : m_theme.borderStrongColor, 1.1));
        painter.setBrush(fill);
        painter.drawPolygon(face.polygon);

        if (face.polygon.boundingRect().width() >= 20.0 && face.polygon.boundingRect().height() >= 14.0)
        {
            painter.setPen(face.face == m_hoveredViewCubeFace ? m_theme.accentTextColor : m_theme.textPrimaryColor);
            painter.drawText(face.polygon.boundingRect(), Qt::AlignCenter, face.label);
        }
    }

    const std::array<std::pair<ViewCubeFace, QRect>, 5> navigationAreas =
    {{
        { ViewCubeFace::Back, QRect(panelRect.left() + 34, panelRect.top() + 2, 48, kNavigationStrip) },
        { ViewCubeFace::Front, QRect(panelRect.left() + 34, panelRect.bottom() - kNavigationStrip - 1, 48, kNavigationStrip) },
        { ViewCubeFace::Left, QRect(panelRect.left() + 2, panelRect.top() + 34, kNavigationStrip, 48) },
        { ViewCubeFace::Right, QRect(panelRect.right() - kNavigationStrip - 1, panelRect.top() + 34, kNavigationStrip, 48) },
        { ViewCubeFace::Bottom, QRect(panelRect.right() - 24, panelRect.bottom() - 22, 20, 18) }
    }};
    const auto faceText = [](ViewCubeFace face)
    {
        switch (face)
        {
        case ViewCubeFace::Front: return QStringLiteral("前");
        case ViewCubeFace::Back: return QStringLiteral("后");
        case ViewCubeFace::Left: return QStringLiteral("左");
        case ViewCubeFace::Right: return QStringLiteral("右");
        case ViewCubeFace::Bottom: return QStringLiteral("下");
        default: return QString();
        }
    };

    for (const auto& [face, area] : navigationAreas)
    {
        if (face == m_hoveredViewCubeFace)
        {
            painter.setPen(Qt::NoPen);
            painter.setBrush(m_theme.hoverBackgroundColor);
            painter.drawRoundedRect(area, 4.0, 4.0);
        }

        painter.setPen(face == m_hoveredViewCubeFace ? m_theme.accentColor : m_theme.textSecondaryColor);
        painter.drawText(area, Qt::AlignCenter, faceText(face));
    }
}

void CadViewer::updateViewCubeHover(const QPoint& screenPos)
{
    ViewCubeFace hoveredFace = ViewCubeFace::None;
    const QVector<ViewCubeFaceOverlay> faces = buildViewCubeFaces();

    for (auto it = faces.crbegin(); it != faces.crend(); ++it)
    {
        if (it->polygon.containsPoint(screenPos, Qt::OddEvenFill))
        {
            hoveredFace = it->face;
            break;
        }
    }

    if (hoveredFace == ViewCubeFace::None && !faces.isEmpty())
    {
        const QRect panelRect
        (
            width() - kCubePanelMargin - kCubePanelSize,
            kCubePanelMargin,
            kCubePanelSize,
            kCubePanelSize
        );

        if (QRect(panelRect.left() + 34, panelRect.top() + 2, 48, kNavigationStrip).contains(screenPos))
            hoveredFace = ViewCubeFace::Back;
        else if (QRect(panelRect.left() + 34, panelRect.bottom() - kNavigationStrip - 1, 48, kNavigationStrip).contains(screenPos))
            hoveredFace = ViewCubeFace::Front;
        else if (QRect(panelRect.left() + 2, panelRect.top() + 34, kNavigationStrip, 48).contains(screenPos))
            hoveredFace = ViewCubeFace::Left;
        else if (QRect(panelRect.right() - kNavigationStrip - 1, panelRect.top() + 34, kNavigationStrip, 48).contains(screenPos))
            hoveredFace = ViewCubeFace::Right;
        else if (QRect(panelRect.right() - 24, panelRect.bottom() - 22, 20, 18).contains(screenPos))
            hoveredFace = ViewCubeFace::Bottom;
    }

    m_hoveredViewCubeFace = hoveredFace;
}

bool CadViewer::handleViewCubeClick(const QPoint& screenPos)
{
    updateViewCubeHover(screenPos);

    if (m_hoveredViewCubeFace == ViewCubeFace::None)
    {
        return false;
    }

    applyViewCubeFace(m_hoveredViewCubeFace);
    return true;
}

void CadViewer::applyViewCubeFace(ViewCubeFace face)
{
    QVector3D forward;
    QVector3D up(0.0f, 0.0f, 1.0f);
    bool planarTopView = false;

    switch (face)
    {
    case ViewCubeFace::Top:
        forward = QVector3D(0.0f, 0.0f, -1.0f);
        up = QVector3D(0.0f, 1.0f, 0.0f);
        planarTopView = true;
        break;
    case ViewCubeFace::Bottom:
        forward = QVector3D(0.0f, 0.0f, 1.0f);
        up = QVector3D(0.0f, 1.0f, 0.0f);
        break;
    case ViewCubeFace::Front:
        forward = QVector3D(0.0f, 1.0f, 0.0f);
        break;
    case ViewCubeFace::Back:
        forward = QVector3D(0.0f, -1.0f, 0.0f);
        break;
    case ViewCubeFace::Left:
        forward = QVector3D(1.0f, 0.0f, 0.0f);
        break;
    case ViewCubeFace::Right:
        forward = QVector3D(-1.0f, 0.0f, 0.0f);
        break;
    default:
        return;
    }

    m_viewInteractionController.setStandardView(m_camera, forward, up, planarTopView);
    invalidateSnapCache();
    update();
}
