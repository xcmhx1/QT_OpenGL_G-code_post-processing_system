// 声明 CadStatusPaneWidget 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 状态栏模块，负责显示当前鼠标世界坐标和后续状态位预留区域。
#pragma once

#include "AppTheme.h"

#include <QLabel>
#include <QPushButton>
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

private:
    QLabel* m_coordinateValueLabel = nullptr;
    QPushButton* m_basePointSnapButton = nullptr;
    QPushButton* m_controlPointSnapButton = nullptr;
    QPushButton* m_gridSnapButton = nullptr;
};
