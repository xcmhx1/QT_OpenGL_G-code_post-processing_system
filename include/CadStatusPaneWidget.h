// 声明 CadStatusPaneWidget 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 状态栏模块，负责显示当前鼠标世界坐标和后续状态位预留区域。
#pragma once

#include "AppTheme.h"

#include <QLabel>
#include <QWidget>
#include <QVector3D>

class CadStatusPaneWidget : public QWidget
{
public:
    explicit CadStatusPaneWidget(QWidget* parent = nullptr);

    void setWorldPosition(const QVector3D& worldPos);
    void setTheme(const AppThemeColors& theme);

private:
    QLabel* m_coordinateValueLabel = nullptr;
};
