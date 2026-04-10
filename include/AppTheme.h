#pragma once

#include <QColor>
#include <QPalette>

enum class AppThemeMode
{
    Light,
    Dark
};

struct AppThemeColors
{
    bool dark = false;
    QColor windowBackground;
    QColor panelBackground;
    QColor surfaceBackground;
    QColor surfaceAltBackground;
    QColor borderColor;
    QColor borderStrongColor;
    QColor textPrimaryColor;
    QColor textSecondaryColor;
    QColor accentColor;
    QColor accentTextColor;
    QColor hoverBackgroundColor;
    QColor pressedBackgroundColor;
    QColor viewerBackgroundColor;
    QColor viewerGridColor;
    QColor processLabelFillColor;
    QColor processLabelBorderColor;
    QColor processLabelTextColor;
    QColor selectedProcessLabelFillColor;
    QColor selectedProcessLabelBorderColor;
    QColor selectedProcessLabelTextColor;
    QColor selectedBasePointColor;
    QColor selectedControlPointColor;
    QColor selectedHandleLabelFillColor;
    QColor selectedHandleLabelBorderColor;
    QColor selectedHandleLabelTextColor;
    QPalette palette;
};

inline AppThemeColors buildAppThemeColors(AppThemeMode mode)
{
    AppThemeColors theme;
    theme.dark = mode == AppThemeMode::Dark;

    if (theme.dark)
    {
        theme.windowBackground = QColor(38, 42, 47);
        theme.panelBackground = QColor(44, 49, 55);
        theme.surfaceBackground = QColor(28, 32, 36);
        theme.surfaceAltBackground = QColor(24, 27, 31);
        theme.borderColor = QColor(76, 82, 90);
        theme.borderStrongColor = QColor(98, 106, 116);
        theme.textPrimaryColor = QColor(236, 240, 244);
        theme.textSecondaryColor = QColor(177, 185, 194);
        theme.accentColor = QColor(118, 174, 255);
        theme.accentTextColor = QColor(16, 20, 26);
        theme.hoverBackgroundColor = QColor(58, 64, 72);
        theme.pressedBackgroundColor = QColor(70, 77, 86);
        theme.viewerBackgroundColor = QColor(20, 23, 27);
        theme.viewerGridColor = QColor(68, 74, 82);
        theme.processLabelFillColor = QColor(24, 32, 40, 210);
        theme.processLabelBorderColor = QColor(110, 210, 255, 220);
        theme.processLabelTextColor = QColor(240, 248, 255);
        theme.selectedProcessLabelFillColor = QColor(255, 196, 64, 230);
        theme.selectedProcessLabelBorderColor = QColor(255, 240, 180);
        theme.selectedProcessLabelTextColor = QColor(34, 22, 0);
        theme.selectedBasePointColor = QColor(255, 206, 84);
        theme.selectedControlPointColor = QColor(88, 214, 255);
        theme.selectedHandleLabelFillColor = QColor(30, 36, 42, 226);
        theme.selectedHandleLabelBorderColor = QColor(160, 214, 255, 230);
        theme.selectedHandleLabelTextColor = QColor(243, 248, 252);
    }
    else
    {
        theme.windowBackground = QColor(242, 245, 249);
        theme.panelBackground = QColor(247, 249, 252);
        theme.surfaceBackground = QColor(255, 255, 255);
        theme.surfaceAltBackground = QColor(236, 240, 244);
        theme.borderColor = QColor(203, 209, 216);
        theme.borderStrongColor = QColor(176, 184, 193);
        theme.textPrimaryColor = QColor(34, 40, 47);
        theme.textSecondaryColor = QColor(96, 104, 114);
        theme.accentColor = QColor(58, 108, 184);
        theme.accentTextColor = QColor(255, 255, 255);
        theme.hoverBackgroundColor = QColor(234, 239, 245);
        theme.pressedBackgroundColor = QColor(223, 230, 238);
        theme.viewerBackgroundColor = QColor(251, 252, 254);
        theme.viewerGridColor = QColor(206, 212, 220);
        theme.processLabelFillColor = QColor(255, 255, 255, 238);
        theme.processLabelBorderColor = QColor(72, 138, 198, 230);
        theme.processLabelTextColor = QColor(28, 36, 44);
        theme.selectedProcessLabelFillColor = QColor(255, 209, 102, 242);
        theme.selectedProcessLabelBorderColor = QColor(214, 155, 34);
        theme.selectedProcessLabelTextColor = QColor(62, 44, 0);
        theme.selectedBasePointColor = QColor(224, 150, 24);
        theme.selectedControlPointColor = QColor(26, 146, 198);
        theme.selectedHandleLabelFillColor = QColor(255, 255, 255, 240);
        theme.selectedHandleLabelBorderColor = QColor(132, 174, 214, 230);
        theme.selectedHandleLabelTextColor = QColor(26, 34, 42);
    }

    QPalette palette;
    palette.setColor(QPalette::Window, theme.windowBackground);
    palette.setColor(QPalette::WindowText, theme.textPrimaryColor);
    palette.setColor(QPalette::Base, theme.surfaceBackground);
    palette.setColor(QPalette::AlternateBase, theme.surfaceAltBackground);
    palette.setColor(QPalette::ToolTipBase, theme.surfaceBackground);
    palette.setColor(QPalette::ToolTipText, theme.textPrimaryColor);
    palette.setColor(QPalette::Text, theme.textPrimaryColor);
    palette.setColor(QPalette::Button, theme.panelBackground);
    palette.setColor(QPalette::ButtonText, theme.textPrimaryColor);
    palette.setColor(QPalette::BrightText, QColor(255, 255, 255));
    palette.setColor(QPalette::Highlight, theme.accentColor);
    palette.setColor(QPalette::HighlightedText, theme.accentTextColor);
    palette.setColor(QPalette::PlaceholderText, theme.textSecondaryColor);
    palette.setColor(QPalette::Light, theme.surfaceBackground);
    palette.setColor(QPalette::Midlight, theme.surfaceAltBackground);
    palette.setColor(QPalette::Mid, theme.borderColor);
    palette.setColor(QPalette::Dark, theme.borderStrongColor);
    palette.setColor(QPalette::Shadow, QColor(0, 0, 0, theme.dark ? 160 : 90));

    const QColor disabledText = theme.dark ? QColor(126, 132, 140) : QColor(146, 152, 160);
    palette.setColor(QPalette::Disabled, QPalette::WindowText, disabledText);
    palette.setColor(QPalette::Disabled, QPalette::Text, disabledText);
    palette.setColor(QPalette::Disabled, QPalette::ButtonText, disabledText);

    theme.palette = palette;
    return theme;
}
