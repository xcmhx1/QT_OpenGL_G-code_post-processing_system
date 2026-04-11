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
#include <QtGlobal>

class CadStatusPaneWidget : public QWidget
{
    Q_OBJECT

public:
    enum SnapOptionBit : quint32
    {
        BasePointSnapBit = 1u << 0,
        ControlPointSnapBit = 1u << 1,
        EndpointSnapBit = 1u << 2,
        MidpointSnapBit = 1u << 3,
        CenterSnapBit = 1u << 4,
        IntersectionSnapBit = 1u << 5,
        GridSnapBit = 1u << 6
    };

    explicit CadStatusPaneWidget(QWidget* parent = nullptr);

    void setWorldPosition(const QVector3D& worldPos);
    void setTheme(const AppThemeColors& theme);
    quint32 snapOptionMask() const;
    void setSnapOptionMask(quint32 mask);

    static constexpr quint32 allSnapOptionMask()
    {
        return BasePointSnapBit
            | ControlPointSnapBit
            | EndpointSnapBit
            | MidpointSnapBit
            | CenterSnapBit
            | IntersectionSnapBit
            | GridSnapBit;
    }

    static constexpr quint32 defaultSnapOptionMask()
    {
        return BasePointSnapBit
            | ControlPointSnapBit
            | EndpointSnapBit
            | MidpointSnapBit
            | CenterSnapBit
            | GridSnapBit;
    }

signals:
    void basePointSnapToggled(bool enabled);
    void controlPointSnapToggled(bool enabled);
    void gridSnapToggled(bool enabled);
    void endpointSnapToggled(bool enabled);
    void midpointSnapToggled(bool enabled);
    void centerSnapToggled(bool enabled);
    void intersectionSnapToggled(bool enabled);
    void snapOptionMaskChanged(quint32 mask);

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
    bool m_applyingSnapMask = false;
};
