#pragma once

#include <QLabel>
#include <QWidget>
#include <QVector3D>

class CadStatusPaneWidget : public QWidget
{
public:
    explicit CadStatusPaneWidget(QWidget* parent = nullptr);

    void setWorldPosition(const QVector3D& worldPos);

private:
    QLabel* m_coordinateValueLabel = nullptr;
};
