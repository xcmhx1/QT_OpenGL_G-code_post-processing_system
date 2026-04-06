#pragma once

#include <memory>
#include <unordered_map>

#include <QColor>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QKeyEvent>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions_4_5_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPoint>
#include <QQuaternion>
#include <QVector3D>
#include <QWheelEvent>

#include "CadController.h"

///前向声明
// 图元类
class CadItem;
// 数据层类
class CadDocument;
class CadEditer;

using EntityId = quintptr;

// 轨道相机：
// - target 为观察目标点
// - distance 为相机眼点到 target 的距离
// - orientation 表示相机朝向
// - viewHeight 控制正交投影视口高度
struct OrbitalCamera
{
    // 当前观察中心/目标点。
    QVector3D target = { 0.0f, 0.0f, 0.0f };

    // 相机眼点到目标点的距离。
    // 在当前实现里主要用于 eyePosition() 计算。
    float distance = 500.0f;

    // 正交投影窗口的世界高度。
    // 值越小表示放大，值越大表示缩小。
    float viewHeight = 200.0f;

    // 正交投影近远裁剪面。
    float nearPlane = -100000.0f;
    float farPlane = 100000.0f;

    // 相机朝向四元数。
    // 约定局部前方向为 -Z，右方向为 +X，上方向为 +Y。
    QQuaternion orientation = QQuaternion(1.0f, 0.0f, 0.0f, 0.0f);

    // 当视线越过水平线后，用于提示 Z 轴改为虚线显示，
    // 帮助用户区分当前是“从上往下看”还是“从下往上看”。
    bool m_axesSwapped = false;

    // 缩放范围保护，避免视口过小或过大导致数值异常。
    static constexpr float kMinViewHeight = 0.001f;
    static constexpr float kMaxViewHeight = 1.0e7f;

    // 计算相机眼点世界坐标。
    QVector3D eyePosition() const;

    // 计算当前相机前方向（世界坐标）。
    QVector3D forwardDirection() const;

    // 计算当前相机右方向（世界坐标）。
    QVector3D rightDirection() const;

    // 计算当前相机上方向（世界坐标）。
    QVector3D upDirection() const;

    // 构造观察矩阵。
    QMatrix4x4 viewMatrix() const;

    // 构造正交投影矩阵。
    QMatrix4x4 projectionMatrix(float aspectRatio) const;

    // 构造 VP 矩阵。
    QMatrix4x4 viewProjectionMatrix(float aspectRatio) const;

    // 轨道旋转：
    // deltaAzimuth   对应水平转动（绕世界上方向/约定参考轴）
    // deltaElevation 对应俯仰转动
    void orbit(float deltaAzimuth, float deltaElevation);

    // 平移相机目标点。
    // worldDx / worldDy 是沿相机右/上方向的世界位移量。
    void pan(float worldDx, float worldDy);

    // 以中心为基准缩放视口。
    void zoom(float factor);

    // 以某个世界锚点为基准缩放视口，使缩放更符合鼠标所指位置。
    void zoomAtPoint(float factor, const QVector3D& worldAnchor);

    // 根据场景包围盒自动调整 target、distance、viewHeight。
    void fitAll(const QVector3D& sceneMin, const QVector3D& sceneMax, float aspectRatio);

    // 重置到二维顶视图。
    // 通常用于 CAD 初始状态或用户快速回正。
    void resetTo2DTopView();

    // 从 2D 顶视图切入一个默认 3D 观察角度。
    void enter3DFrom2D();

    // 返回当前是否处于“越过水平线”的状态。
    bool axesSwapped() const { return m_axesSwapped; }

    // 根据当前 forward 方向刷新 m_axesSwapped。
    void updateAxesSwappedState();
};

// 单个实体对应的一组 GPU 资源。
struct EntityGpuBuffer
{
    // 顶点缓冲。
    QOpenGLBuffer vbo{ QOpenGLBuffer::VertexBuffer };

    // 顶点数组对象，封装顶点属性绑定状态。
    QOpenGLVertexArrayObject vao;

    // 顶点数量。
    int vertexCount = 0;

    // OpenGL 图元类型，如 GL_LINES / GL_LINE_STRIP / GL_POINTS。
    GLenum primitiveType = GL_LINE_STRIP;

    // 实体绘制颜色（RGB，0~1）。
    QVector3D color = { 1.0f, 1.0f, 1.0f };
};

struct TransientPrimitive
{
    QVector<QVector3D> vertices;
    GLenum primitiveType = GL_LINE_STRIP;
    QVector3D color = { 0.25f, 0.85f, 1.0f };
    float pointSize = 1.0f;
    bool roundPoint = false;
};

enum class ViewInteractionMode
{
    Idle,
    Orbiting,
    Panning,
};

enum class CameraViewMode
{
    Planar2D,
    Orbit3D,
};

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
    // 初始化着色器程序。
    void initShaders();

    // 初始化网格缓冲。
    void initGridBuffer();

    // 初始化坐标轴缓冲。
    void initAxisBuffer();

    // 初始化轨道中心标记缓冲。
    void initOrbitMarkerBuffer();
    void initTransientBuffer();

    // 上传单个实体到 GPU。
    void uploadEntity(const CadItem* entity);

    // 删除指定实体对应的 GPU 缓冲。
    void removeEntityBuffer(EntityId id);

    // 重建全部实体 GPU 缓冲。
    void rebuildAllBuffers();

    // 清空全部实体 GPU 缓冲。
    void clearAllBuffers();

    // 绘制背景网格。
    void renderGrid();

    // 绘制坐标轴。
    void renderAxis();

    // 绘制场景实体。
    void renderEntities();

    // 绘制轨道旋转中心标记。
    void renderOrbitMarker();
    void renderTransientPrimitives();

    // 重新计算场景包围盒和轨道中心。
    void updateSceneBounds();
    void handleDocumentSceneChanged();
    void updateHoveredWorldPosition(const QPoint& screenPos);
    QVector3D screenToWorkPlane(const QPoint& screenPos, float planeZ) const;
    void appendCrosshairPrimitives(std::vector<TransientPrimitive>& primitives) const;

    // 围绕场景中心进行轨道旋转。
    void orbitCameraAroundSceneCenter(float deltaAzimuth, float deltaElevation);

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
    // 当前场景文档，不拥有其生命周期。
    CadDocument* m_scene = nullptr;
    CadEditer* m_editer = nullptr;

    // 视图相机。
    OrbitalCamera m_camera;

    // 实体着色器。
    std::unique_ptr<QOpenGLShaderProgram> m_entityShader;

    // 网格/坐标轴着色器。
    std::unique_ptr<QOpenGLShaderProgram> m_gridShader;

    // 各实体对应的 GPU 缓冲表。
    std::unordered_map<EntityId, EntityGpuBuffer> m_entityBuffers;

    // 网格 VBO / VAO。
    QOpenGLBuffer m_gridVbo{ QOpenGLBuffer::VertexBuffer };
    QOpenGLVertexArrayObject m_gridVao;
    int m_gridVertexCount = 0;

    // 坐标轴 VBO / VAO。
    QOpenGLBuffer m_axisVbo{ QOpenGLBuffer::VertexBuffer };
    QOpenGLVertexArrayObject m_axisVao;

    // X/Y 坐标轴顶点数量固定为 4（两条线段）。
    // Z 轴根据相机状态在“实线段”和“虚线段”之间切换绘制。
    int m_axisXyVertexCount = 4;
    int m_axisZSolidOffset = 4;
    int m_axisZSolidVertexCount = 2;
    int m_axisZDashedOffset = 0;
    int m_axisZDashedVertexCount = 0;

    // 轨道中心标记缓冲。
    QOpenGLBuffer m_orbitMarkerVbo{ QOpenGLBuffer::VertexBuffer };
    QOpenGLVertexArrayObject m_orbitMarkerVao;
    QOpenGLBuffer m_transientVbo{ QOpenGLBuffer::VertexBuffer };
    QOpenGLVertexArrayObject m_transientVao;

    // 当前视口尺寸。
    int m_viewportWidth = 1;
    int m_viewportHeight = 1;

    // OpenGL 是否已完成初始化。
    bool m_glInitialized = false;

    // 实体缓冲是否需要重建。
    bool m_buffersDirty = true;

    // 当前是否已有有效场景包围盒。
    bool m_hasSceneBounds = false;

    // 场景包围盒最小点/最大点。
    QVector3D m_sceneMin;
    QVector3D m_sceneMax;

    // 场景轨道旋转中心，一般取包围盒中心。
    QVector3D m_orbitCenter;

    CameraViewMode m_viewMode = CameraViewMode::Planar2D;
    ViewInteractionMode m_interactionMode = ViewInteractionMode::Idle;
    bool m_ignoreNextOrbitDelta = false;
    EntityId m_selectedEntityId = 0;
    QPoint m_cursorScreenPos;
    bool m_showCrosshairOverlay = false;
    float m_crosshairPlaneZ = 0.0f;
    QMetaObject::Connection m_sceneChangedConnection;

    // 控制器负责接收 Viewer 输入并维护绘图状态。
    CadController m_controller;
};
