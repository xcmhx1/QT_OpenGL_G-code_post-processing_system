#pragma once

#include <QVector3D>

class CadItem;

struct CadProcessVisualInfo
{
    bool valid = false;
    bool closedPath = false;
    int processOrder = -1;
    bool isReverse = false;
    QVector3D forwardStartPoint;
    QVector3D forwardEndPoint;
    QVector3D startPoint;
    QVector3D endPoint;
    QVector3D direction;
    QVector3D labelAnchor;
};

bool isProcessVisualizable(const CadItem* item);

CadProcessVisualInfo buildProcessVisualInfo(const CadItem* item);
