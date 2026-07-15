// 声明 CadAppearanceSettingsDialog 模块，对外暴露自定义外观设置对话框。
#pragma once

#include "desktop/AppTheme.h"

#include <QDialog>
#include <QMap>

class QComboBox;
class QPushButton;

class CadAppearanceSettingsDialog : public QDialog
{
public:
    explicit CadAppearanceSettingsDialog(AppThemeMode baseMode, const AppThemeColors& initialTheme, QWidget* parent = nullptr);

    AppThemeMode baseMode() const;
    AppThemeColors themeColors() const;

private:
    void buildUi();
    void rebuildFromBaseMode(AppThemeMode mode);
    void updateColorButtons();
    QPushButton* addColorButton(QWidget* parent, const QString& label, const QString& key);

private:
    AppThemeMode m_baseMode = AppThemeMode::Light;
    AppThemeColors m_theme;
    QComboBox* m_baseModeComboBox = nullptr;
    QMap<QString, QPushButton*> m_colorButtons;
};
