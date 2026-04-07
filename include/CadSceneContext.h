// CadSceneContext 头文件
// 声明 CadSceneContext 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 场景上下文模块，负责文档绑定、场景边界和实体遍历等基础上下文能力。
#pragma once

#include <QMetaObject>
#include <QVector3D>

#include "CadDocument.h"

class CadSceneContext
{
public:
    CadSceneContext() = default;

    // 析构时断开已绑定的文档信号连接
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

    // 获取当前绑定的文档对象
    // @return 当前文档指针，未绑定时返回 nullptr
    CadDocument* document() const;

    // 刷新场景包围盒与轨道中心
    void refreshBounds();

    // 查询当前场景是否有有效包围盒
    // @return 如果已有有效场景边界返回 true，否则返回 false
    bool hasBounds() const;

    // 获取场景包围盒最小点
    // @return 包围盒最小点引用
    const QVector3D& minPoint() const;

    // 获取场景包围盒最大点
    // @return 包围盒最大点引用
    const QVector3D& maxPoint() const;

    // 获取轨道观察中心
    // @return 轨道观察中心引用
    const QVector3D& orbitCenter() const;

private:
    // 当前绑定的文档对象
    CadDocument* m_document = nullptr;

    // 文档 sceneChanged 信号的连接句柄
    QMetaObject::Connection m_sceneChangedConnection;

    // 当前场景是否存在有效包围盒
    bool m_hasBounds = false;

    // 场景包围盒最小点
    QVector3D m_minPoint;

    // 场景包围盒最大点
    QVector3D m_maxPoint;

    // 当前轨道观察中心
    QVector3D m_orbitCenter;
};
