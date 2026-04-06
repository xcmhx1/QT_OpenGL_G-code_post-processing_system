#pragma once

#include <QMetaObject>
#include <QVector3D>

#include "CadDocument.h"

class CadSceneContext
{
public:
    CadSceneContext() = default;
    ~CadSceneContext();

    template<typename Receiver, typename Method>
    void bindDocument(CadDocument* document, Receiver* receiver, Method method)
    {
        if (m_document != nullptr)
        {
            QObject::disconnect(m_sceneChangedConnection);
        }

        m_document = document;
        m_sceneChangedConnection = QMetaObject::Connection();

        if (m_document != nullptr)
        {
            m_sceneChangedConnection = QObject::connect(m_document, &CadDocument::sceneChanged, receiver, method);
        }

        refreshBounds();
    }

    CadDocument* document() const;
    void refreshBounds();

    bool hasBounds() const;
    const QVector3D& minPoint() const;
    const QVector3D& maxPoint() const;
    const QVector3D& orbitCenter() const;

private:
    CadDocument* m_document = nullptr;
    QMetaObject::Connection m_sceneChangedConnection;
    bool m_hasBounds = false;
    QVector3D m_minPoint;
    QVector3D m_maxPoint;
    QVector3D m_orbitCenter;
};
