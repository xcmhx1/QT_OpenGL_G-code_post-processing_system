#pragma once

#include <QVector>
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

enum class CadSelectionHandleShape
{
    RoundPoint,
    Triangle
};

struct CadSelectionHandleInfo
{
    QVector3D position;
    bool isBasePoint = false;
    CadSelectionHandleShape shape = CadSelectionHandleShape::RoundPoint;
    QVector3D direction;
};

bool isProcessVisualizable(const CadItem* item);

CadProcessVisualInfo buildProcessVisualInfo(const CadItem* item);

QVector<CadSelectionHandleInfo> buildSelectionHandleInfo(const CadItem* item);
