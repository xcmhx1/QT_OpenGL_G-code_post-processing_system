#pragma once

// 声明 CadViewer 模块，对外暴露当前组件的核心类型、接口和协作边界。
// CAD 主视图模块，负责 OpenGL 生命周期、输入接入、场景刷新和信号分发。
#include <memory>

#include <QDragEnterEvent>
#include <QDropEvent>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QOpenGLFunctions_4_5_Core>
#include <QOpenGLWidget>
#include <QPoint>
#include <QVector3D>
#include <QWheelEvent>

#include "CadCamera.h"
#include "CadGraphicsCoordinator.h"
#include "CadRenderTypes.h"
#include "CadSceneCoordinator.h"
#include "CadViewInteractionController.h"
#include "CadController.h"

///前向声明
// 图元类
class CadItem;
// 数据层类
class CadDocument;
class CadEditer;

// CAD 视图窗口：
// - 负责 OpenGL 初始化与绘制
// - 管理相机和视图交互
// - 管理实体 GPU 缓冲
// - 提供简单拾取能力
class CadViewer : public QOpenGLWidget, protected QOpenGLFunctions_4_5_Core
{
    Q_OBJECT

public:
    explicit CadViewer(QWidget* parent = nullptr);
    ~CadViewer() override;

    // 设置当前显示文档/场景。
    void setDocument(CadDocument* document);
    void setEditer(CadEditer* editer);

    // 视图适配整个场景。
    void fitScene();

    // 放大。
    void zoomIn(float factor = 1.4f);

    // 缩小。
    void zoomOut(float factor = 1.4f);

    // 屏幕坐标转世界坐标。
    // depth 为 NDC 深度，常用 -1 表示近平面，+1 表示远平面。
    QVector3D screenToWorld(const QPoint& screenPos, float depth = 0.0f) const;
    QVector3D screenToGroundPlane(const QPoint& screenPos) const;

    // 世界坐标投影到屏幕坐标。
    QPoint worldToScreen(const QVector3D& worldPos) const;

    // 供控制器调用的视图动作接口。
    void beginOrbitInteraction();
    void beginPanInteraction();
    void updateOrbitInteraction(const QPoint& screenDelta);
    void updatePanInteraction(const QPoint& screenDelta);
    void endViewInteraction();
    void selectEntityAt(const QPoint& screenPos);
    void zoomAtScreenPosition(const QPoint& screenPos, float factor);
    void resetToTopView();
    void fitSceneView();
    CameraViewMode viewMode() const;
    ViewInteractionMode interactionMode() const;
    bool shouldIgnoreNextOrbitDelta() const;
    void consumeIgnoreNextOrbitDelta();
    void requestViewUpdate();
    CadItem* selectedEntity() const;
    void appendCommandMessage(const QString& message);
    void refreshCommandPrompt();

signals:
    void hoveredWorldPositionChanged(const QVector3D& worldPos);
    void commandPromptChanged(const QString& prompt);
    void commandMessageAppended(const QString& message);
    void fileDropRequested(const QString& filePath);

protected:
    // OpenGL 初始化。
    void initializeGL() override;

    // 视口尺寸变化。
    void resizeGL(int w, int h) override;

    // 主绘制入口。
    void paintGL() override;

    // 鼠标按下事件：
    // - 中键 + Shift：轨道旋转
    // - 中键：平移
    // - 左键：拾取实体
    void mousePressEvent(QMouseEvent* event) override;

    // 鼠标移动事件。
    void mouseMoveEvent(QMouseEvent* event) override;

    void leaveEvent(QEvent* event) override;

    // 鼠标释放事件。
    void mouseReleaseEvent(QMouseEvent* event) override;

    // 滚轮缩放事件。
    void wheelEvent(QWheelEvent* event) override;

    // 键盘快捷键处理。
    void keyPressEvent(QKeyEvent* event) override;
    void dragEnterEvent(QDragEnterEvent* event) override;
    void dropEvent(QDropEvent* event) override;

private:
    // 重建全部实体 GPU 缓冲。
    void rebuildAllBuffers();

    // 绘制背景网格与坐标轴。
    void renderGrid(const QMatrix4x4& viewProjection);
    void renderAxis(const QMatrix4x4& viewProjection);

    // 绘制场景实体。
    void renderEntities(const QMatrix4x4& viewProjection);

    // 绘制轨道旋转中心标记与 transient overlay。
    void renderOrbitMarker(const QMatrix4x4& viewProjection);
    void renderTransientPrimitives(const QMatrix4x4& viewProjection);

    void handleDocumentSceneChanged();
    void updateHoveredWorldPosition(const QPoint& screenPos);

    // 简单屏幕空间拾取，返回命中的实体 ID。
    EntityId pickEntity(const QPoint& screenPos) const;
    CadItem* findEntityById(EntityId id) const;
    std::vector<TransientPrimitive> buildTransientPrimitives() const;

    // 当前视口宽高比。
    float aspectRatio() const;

    // 1 个屏幕像素大致对应多少世界单位。
    // 用于拖拽平移时把鼠标位移换算为世界位移。
    float pixelToWorldScale() const;

private:
    // 视图相机。
    OrbitalCamera m_camera;

    // scene coordinator 负责文档绑定、场景边界和 GPU 缓冲协调。
    CadSceneCoordinator m_sceneCoordinator;

    // graphics coordinator 负责 OpenGL 初始化状态、shader 与各渲染器。
    CadGraphicsCoordinator m_graphicsCoordinator;

    // 当前视口尺寸。
    int m_viewportWidth = 1;
    int m_viewportHeight = 1;

    CadViewInteractionController m_viewInteractionController;
    EntityId m_selectedEntityId = 0;
    QPoint m_cursorScreenPos;
    bool m_showCrosshairOverlay = false;
    float m_crosshairPlaneZ = 0.0f;

    // 控制器负责接收 Viewer 输入并维护绘图状态。
    CadController m_controller;
};
