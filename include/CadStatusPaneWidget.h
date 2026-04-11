// 声明 CadStatusPaneWidget 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 状态栏模块，负责显示当前鼠标世界坐标和后续状态位预留区域。
#pragma once

#include "AppTheme.h"

#include <QAction>
#include <QLabel>
#include <QMenu>
#include <QPushButton>
#include <QToolButton>
#include <QWidget>
#include <QVector3D>

class CadStatusPaneWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CadStatusPaneWidget(QWidget* parent = nullptr);

    void setWorldPosition(const QVector3D& worldPos);
    void setTheme(const AppThemeColors& theme);

signals:
    void basePointSnapToggled(bool enabled);
    void controlPointSnapToggled(bool enabled);
    void gridSnapToggled(bool enabled);
    void endpointSnapToggled(bool enabled);
    void midpointSnapToggled(bool enabled);
    void centerSnapToggled(bool enabled);
    void intersectionSnapToggled(bool enabled);

private:
    void refreshSnapSummary();

private:
    QLabel* m_coordinateValueLabel = nullptr;
    QToolButton* m_snapSettingsButton = nullptr;
    QMenu* m_snapSettingsMenu = nullptr;
    QAction* m_basePointSnapAction = nullptr;
    QAction* m_controlPointSnapAction = nullptr;
    QAction* m_gridSnapAction = nullptr;
    QAction* m_endpointSnapAction = nullptr;
    QAction* m_midpointSnapAction = nullptr;
    QAction* m_centerSnapAction = nullptr;
    QAction* m_intersectionSnapAction = nullptr;
};
