#pragma once

// CadViewer 头文件
// 声明 CadViewer 模块，对外暴露当前组件的核心类型、接口和协作边界。
// CAD 主视图模块，负责 OpenGL 生命周期、输入接入、场景刷新和信号分发。

#include <memory>

// Qt 核心模块
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QElapsedTimer>
#include <QEnterEvent>
#include <QFocusEvent>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QOpenGLFunctions_4_5_Core>
#include <QOpenGLWidget>
#include <QPoint>
#include <QSet>
#include <QTimer>
#include <QVector>
#include <QVector3D>
#include <QWheelEvent>

// CAD 模块内部依赖
#include "CadCamera.h"
#include "AppTheme.h"
#include "CadGraphicsCoordinator.h"
#include "CadRenderTypes.h"
#include "CadSceneCoordinator.h"
#include "CadViewInteractionController.h"
#include "CadController.h"

/// 前向声明
// 图元类
class CadItem;
struct CadSelectionHandleInfo;
// 数据层类
class CadDocument;
class CadEditer;

// CAD 视图窗口：
// - 负责 OpenGL 初始化与绘制
// - 管理相机和视图交互
// - 管理实体 GPU 缓冲
// - 提供简单拾取能力
// 继承自 QOpenGLWidget 和 OpenGL 4.5 核心功能
class CadViewer : public QOpenGLWidget, protected QOpenGLFunctions_4_5_Core
{
    Q_OBJECT  // Qt 元对象系统宏，支持信号槽机制

public:
    enum class SelectionUpdateMode
    {
        Replace,
        Toggle
    };

    // 构造函数，初始化 CAD 视图
    explicit CadViewer(QWidget* parent = nullptr);

    // 析构函数，清理资源
    ~CadViewer() override;

    // 设置当前显示文档
    void setDocument(CadDocument* document);

    // 设置当前编辑器
    void setEditer(CadEditer* editer);

    // 设置默认绘图属性。
    void setDefaultDrawingProperties(const QString& layerName, const QColor& color, int colorIndex);

    // 设置基点吸附开关。
    void setBasePointSnapEnabled(bool enabled);

    // 设置控制点吸附开关。
    void setControlPointSnapEnabled(bool enabled);

    // 设置网格点吸附开关。
    void setGridSnapEnabled(bool enabled);

    // 设置 Viewer 主题。
    void setTheme(const AppThemeColors& theme);

    // 开始绘制指定类型图元。
    void startDrawing(DrawType drawType);

    // 对当前选中图元开始移动命令。
    bool startMoveSelected();

    // 视图适配整个场景，调整相机使整个场景可见
    void fitScene();

    // 放大视图
    // @param factor 缩放因子，默认为 1.4
    void zoomIn(float factor = 1.4f);

    // 缩小视图
    // @param factor 缩放因子，默认为 1.4
    void zoomOut(float factor = 1.4f);

    // 屏幕坐标转世界坐标
    // @param screenPos 屏幕坐标点
    // @param depth NDC 深度，常用 -1 表示近平面，+1 表示远平面
    // @return 对应的世界坐标
    QVector3D screenToWorld(const QPoint& screenPos, float depth = 0.0f) const;

    // 屏幕坐标转地面平面坐标
    // @param screenPos 屏幕坐标点
    // @return 对应的地面平面坐标
    QVector3D screenToGroundPlane(const QPoint& screenPos) const;

    // 世界坐标投影到屏幕坐标
    // @param worldPos 世界坐标
    // @return 对应的屏幕坐标
    QPoint worldToScreen(const QVector3D& worldPos) const;

    // 屏幕坐标转交互用世界坐标，并按当前开关执行吸附。
    QVector3D resolveInteractiveWorldPosition(const QPoint& screenPos) const;

    // 供控制器调用的视图动作接口

    // 开始轨道旋转交互
    void beginOrbitInteraction();

    // 开始平移交互
    void beginPanInteraction();

    // 更新轨道旋转交互
    // @param screenDelta 屏幕坐标增量
    void updateOrbitInteraction(const QPoint& screenDelta);

    // 更新平移交互
    // @param screenDelta 屏幕坐标增量
    void updatePanInteraction(const QPoint& screenDelta);

    // 结束视图交互
    void endViewInteraction();

    // 在屏幕位置选择实体
    // @param screenPos 屏幕坐标点
    void selectEntityAt(const QPoint& screenPos, SelectionUpdateMode updateMode = SelectionUpdateMode::Replace);

    // 在屏幕矩形窗口内批量选择实体。
    // crossingSelection=true 为碰选；false 为包含选。
    void selectEntitiesInWindow
    (
        const QPoint& startScreenPos,
        const QPoint& endScreenPos,
        bool crossingSelection,
        SelectionUpdateMode updateMode = SelectionUpdateMode::Replace
    );

    // 获取当前选中的实体集合（按场景遍历顺序）。
    QVector<CadItem*> selectedEntities() const;

    // 显示框选预览窗口。
    void showSelectionWindowPreview(const QPoint& anchorScreenPos, const QPoint& currentScreenPos);

    // 隐藏框选预览窗口。
    void hideSelectionWindowPreview();

    // 在当前选中实体上命中控制点/基点手柄。
    bool pickSelectedHandle(const QPoint& screenPos, CadSelectionHandleInfo* outHandle = nullptr) const;

    // 在指定屏幕位置缩放
    // @param screenPos 屏幕坐标点
    // @param factor 缩放因子
    void zoomAtScreenPosition(const QPoint& screenPos, float factor);

    // 重置到顶视图
    void resetToTopView();

    // 适配场景视图
    void fitSceneView();

    // 获取当前视图模式
    CameraViewMode viewMode() const;

    // 获取当前交互模式
    ViewInteractionMode interactionMode() const;

    // 是否应该忽略下一次轨道旋转增量
    bool shouldIgnoreNextOrbitDelta() const;

    // 消耗忽略下一次轨道旋转增量的标志
    void consumeIgnoreNextOrbitDelta();

    // 请求视图更新
    void requestViewUpdate();

    // 获取当前选中的实体
    CadItem* selectedEntity() const;

    // 清空当前选中实体。
    void clearSelection();

    // 追加命令消息
    // @param message 消息内容
    void appendCommandMessage(const QString& message);

    // 刷新命令提示
    void refreshCommandPrompt();

signals:
    // 鼠标悬停世界位置变化信号
    // @param worldPos 新的世界坐标
    void hoveredWorldPositionChanged(const QVector3D& worldPos);

    // 命令提示变化信号
    // @param prompt 新的命令提示
    void commandPromptChanged(const QString& prompt);

    // 命令消息追加信号
    // @param message 追加的消息
    void commandMessageAppended(const QString& message);

    // 当前选中图元变化信号。
    void selectedEntityChanged(CadItem* item);

    // 文件拖放请求信号
    // @param filePath 文件路径
    void fileDropRequested(const QString& filePath);

protected:
    // OpenGL 初始化，在窗口第一次显示时调用
    void initializeGL() override;

    // 视口尺寸变化响应
    // @param w 新宽度
    // @param h 新高度
    void resizeGL(int w, int h) override;

    // 主绘制入口，每帧调用
    void paintGL() override;

    // 鼠标按下事件处理：
    // - 中键 + Shift：轨道旋转
    // - 中键：平移
    // - 左键：拾取实体
    // @param event 鼠标事件
    void mousePressEvent(QMouseEvent* event) override;

    // 鼠标移动事件处理
    // @param event 鼠标事件
    void mouseMoveEvent(QMouseEvent* event) override;

    // 鼠标离开事件处理
    // @param event 离开事件
    void leaveEvent(QEvent* event) override;

    // 鼠标进入 Viewer 时重新隐藏系统光标
    void enterEvent(QEnterEvent* event) override;

    // 焦点回到 Viewer 时重新隐藏系统光标
    void focusInEvent(QFocusEvent* event) override;

    // 鼠标释放事件处理
    // @param event 鼠标事件
    void mouseReleaseEvent(QMouseEvent* event) override;

    // 滚轮缩放事件处理
    // @param event 滚轮事件
    void wheelEvent(QWheelEvent* event) override;

    // 键盘快捷键处理
    // @param event 键盘事件
    void keyPressEvent(QKeyEvent* event) override;

    // 禁止 Tab/Shift+Tab 触发焦点跳转，保证其可用于 CAD 交互。
    bool focusNextPrevChild(bool next) override;

    // 拖拽进入事件处理
    // @param event 拖拽事件
    void dragEnterEvent(QDragEnterEvent* event) override;

    // 拖放事件处理
    // @param event 拖放事件
    void dropEvent(QDropEvent* event) override;

private:
    // 重建全部实体 GPU 缓冲
    void rebuildAllBuffers();

    // 统一重新应用空白光标，避免菜单/焦点切换后系统光标回显
    void ensureBlankCursor();

    // 绘制背景网格
    // @param viewProjection 视图投影矩阵
    void renderGrid(const QMatrix4x4& viewProjection);

    // 绘制坐标轴
    // @param viewProjection 视图投影矩阵
    void renderAxis(const QMatrix4x4& viewProjection);

    // 绘制场景实体
    // @param viewProjection 视图投影矩阵
    void renderEntities(const QMatrix4x4& viewProjection);

    // 绘制轨道旋转中心标记
    // @param viewProjection 视图投影矩阵
    void renderOrbitMarker(const QMatrix4x4& viewProjection);

    // 绘制临时覆盖图元
    // @param viewProjection 视图投影矩阵
    void renderTransientPrimitives(const QMatrix4x4& viewProjection);

    // 绘制加工顺序编号
    void renderProcessOrderLabels();

    // 绘制选中/框选候选图元的叠加高亮（AutoCAD 风格）。
    void renderEntitySelectionOverlays();

    // 绘制框选窗口预览。
    void renderSelectionWindowPreview();

    // 处理文档场景变化
    void handleDocumentSceneChanged();

    // 更新悬停世界位置
    // @param screenPos 屏幕坐标
    void updateHoveredWorldPosition(const QPoint& screenPos);

    // 将原始地面平面点按当前吸附配置修正到目标点。
    QVector3D applySnapToGroundPosition
    (
        const QPoint& screenPos,
        const QVector3D& worldPos,
        bool* snapped = nullptr,
        bool* objectSnap = nullptr
    ) const;

    // 构建当前吸附命中的高亮 overlay 图元。
    std::vector<TransientPrimitive> buildSnapHighlightPrimitives() const;

    // 计算当前视口在地平面上的包围范围。
    void computeVisibleGroundBounds(float& minX, float& maxX, float& minY, float& maxY) const;

    // 计算当前缩放等级下的动态网格步长。
    float currentGridStep() const;

    // 更新当前选中实体并在变化时发出信号。
    void setSelectedEntityId(EntityId entityId);

    // 批量设置当前选中实体集合并同步主选中实体。
    void setSelectedEntities(const QSet<EntityId>& entityIds, EntityId preferredEntityId = 0);

    // 简单屏幕空间拾取，返回命中的实体 ID
    // @param screenPos 屏幕坐标
    // @return 命中的实体ID，0 表示未命中
    EntityId pickEntity(const QPoint& screenPos) const;

    // 根据 ID 查找实体
    // @param id 实体ID
    // @return 实体指针，nullptr 表示未找到
    CadItem* findEntityById(EntityId id) const;

    // 构建临时图元列表
    // @return 临时图元列表
    std::vector<TransientPrimitive> buildTransientPrimitives() const;

    // 构建加工方向箭头图元
    // @return 加工方向 overlay 图元列表
    std::vector<TransientPrimitive> buildProcessDirectionPrimitives() const;

    // 构建选中图元的基点/控制点图元
    // @return 选中态手柄 overlay 图元列表
    std::vector<TransientPrimitive> buildSelectedEntityHandlePrimitives() const;

    // 刷新框选预览阶段的命中实体集合。
    void updateSelectionWindowPreviewCandidates();

    // 重置重叠夹点悬停选择状态。
    void resetOverlappedHandleHoverState();

    // 更新重叠夹点悬停选择状态。
    void updateOverlappedHandleHoverState(const QPoint& screenPos);

    // 根据当前重叠候选状态解析需要高亮的句柄索引。
    int resolveHoveredHandleIndex(const QVector<CadSelectionHandleInfo>& handles) const;

    // 处理重叠夹点选择框内的鼠标按下。
    bool handleOverlappedHandlePopupPress(const QPoint& screenPos);

    // 循环切换重叠夹点候选。
    bool cycleOverlappedHandleCandidate(int step);

    // 绘制重叠夹点选择框。
    void renderOverlappedHandlePopup();

    // 绘制动态输入浮框。
    void renderDynamicInputOverlay();

    // 绘制动态命令浮框。
    void renderDynamicCommandOverlay();

    // 获取当前视口宽高比
    // @return 宽高比（宽度/高度）
    float aspectRatio() const;

    // 计算像素到世界的缩放比例
    // 1 个屏幕像素大致对应多少世界单位
    // 用于拖拽平移时把鼠标位移换算为世界位移
    // @return 缩放比例
    float pixelToWorldScale() const;

private:
    // 视图相机，管理观察变换
    OrbitalCamera m_camera;

    // 场景协调器，负责文档绑定、场景边界和 GPU 缓冲协调
    CadSceneCoordinator m_sceneCoordinator;

    // 图形协调器，负责 OpenGL 初始化状态、shader 与各渲染器
    CadGraphicsCoordinator m_graphicsCoordinator;

    // 当前视口宽度
    int m_viewportWidth = 1;

    // 当前视口高度
    int m_viewportHeight = 1;

    // 视图交互控制器，管理用户交互逻辑
    CadViewInteractionController m_viewInteractionController;

    // 当前选中实体的 ID，0 表示无选中
    EntityId m_selectedEntityId = 0;

    // 当前选中实体集合，用于后续批量编辑能力扩展。
    QSet<EntityId> m_selectedEntityIds;

    // 框选拖拽预览阶段的候选实体集合（鼠标释放前实时更新）。
    QSet<EntityId> m_windowPreviewEntityIds;

    // 框选预览状态。
    struct SelectionWindowPreviewState
    {
        bool visible = false;
        bool crossingSelection = false;
        QPoint anchorScreenPos;
        QPoint currentScreenPos;
    } m_selectionWindowPreview;

    // 当前光标屏幕位置
    QPoint m_cursorScreenPos;

    // 是否显示十字准线覆盖
    bool m_showCrosshairOverlay = false;

    // 十字准线平面 Z 坐标
    float m_crosshairPlaneZ = 0.0f;

    // 控制器，负责接收 Viewer 输入并维护绘图状态
    CadController m_controller;

    // 当前主题颜色
    AppThemeColors m_theme = buildAppThemeColors(AppThemeMode::Light);

    // 基点吸附开关
    bool m_basePointSnapEnabled = false;

    // 控制点吸附开关
    bool m_controlPointSnapEnabled = false;

    // 网格点吸附开关
    bool m_gridSnapEnabled = false;

    // 重叠夹点候选状态。
    struct OverlappedHandleHoverState
    {
        EntityId entityId = 0;
        QVector<int> candidateIndices;
        int activeCandidateOrdinal = 0;
        QPoint anchorScreenPos;
        bool popupVisible = false;
        QElapsedTimer hoverTimer;
    } m_overlappedHandleHoverState;

    // 重叠夹点弹框延迟定时器（单次触发）。
    QTimer m_overlappedHandlePopupTimer;
};


