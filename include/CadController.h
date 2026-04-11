// CadController 头文件
// 声明 CadController 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 输入控制模块，负责解释键盘、鼠标、滚轮事件并驱动绘图/编辑命令。

#pragma once

// Qt 核心模块
#include <QColor>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QString>
#include <QStringList>
#include <QVector3D>
#include <QWheelEvent>

// CAD 模块内部依赖
#include "DrawStateMachine.h"

// 前向声明
class CadViewer;
class CadEditer;

// Viewer 动态输入浮框展示数据。
struct CadDynamicInputOverlayState
{
    bool visible = false;
    QString title;
    QString stageHint;
    QString xValueText;
    QString yValueText;
    bool xLocked = false;
    bool yLocked = false;
    bool xActive = false;
    bool yActive = false;
    bool expressionMode = false;
    QString expressionText;
};

// Viewer 动态命令浮框展示数据。
struct CadDynamicCommandOverlayState
{
    bool visible = false;
    QString inputText;
    QString hintText;
    QStringList candidates;
    int activeCandidateIndex = 0;
};

// CAD 控制器类：
// 负责解释用户输入（键盘、鼠标、滚轮事件），
// 将其转换为具体的绘图/编辑命令，并管理绘图状态机。
class CadController
{
public:
    // 设置关联的视图对象
    // @param viewer CAD 视图对象指针
    void setViewer(CadViewer* viewer);

    // 设置关联的编辑器对象
    // @param editer CAD 编辑器对象指针
    void setEditer(CadEditer* editer);

    // 重置控制器状态
    void reset();

    // 更新默认绘图属性。
    void setDefaultDrawingProperties(const QString& layerName, const QColor& color, int colorIndex);

    // 开始绘制指定类型的图元
    // @param primitiveKind 绘制类型（如直线、圆等）
    // @param color 图元颜色，默认为白色
    void beginDrawing(DrawType primitiveKind, const QColor& color = QColor(255, 255, 255));

    // 开始移动当前选中实体。
    bool beginMoveSelected();

    // 取消当前绘制操作
    void cancelDrawing();

    // 处理鼠标按下事件
    // @param event 鼠标事件
    // @return 如果事件被处理返回 true，否则返回 false
    bool handleMousePress(QMouseEvent* event);

    // 处理鼠标移动事件
    // @param event 鼠标事件
    // @return 如果事件被处理返回 true，否则返回 false
    bool handleMouseMove(QMouseEvent* event);

    // 处理鼠标释放事件
    // @param event 鼠标事件
    // @return 如果事件被处理返回 true，否则返回 false
    bool handleMouseRelease(QMouseEvent* event);

    // 处理滚轮事件
    // @param event 滚轮事件
    // @return 如果事件被处理返回 true，否则返回 false
    bool handleWheel(QWheelEvent* event);

    // 处理键盘按下事件
    // @param event 键盘事件
    // @return 如果事件被处理返回 true，否则返回 false
    bool handleKeyPress(QKeyEvent* event);

    // 获取绘图状态机（可修改）
    // @return 绘图状态机引用
    DrawStateMachine& drawState();

    // 获取绘图状态机（只读）
    // @return 绘图状态机常量引用
    const DrawStateMachine& drawState() const;

    // 获取当前命令提示文本
    // @return 命令提示字符串
    QString currentPrompt() const;

    // 获取当前命令名称
    // @return 命令名称字符串
    QString currentCommandName() const;

    // 查询当前动态输入浮框展示状态。
    CadDynamicInputOverlayState dynamicInputOverlayState() const;

    // 查询当前动态命令浮框展示状态。
    CadDynamicCommandOverlayState dynamicCommandOverlayState() const;

private:
    // 开始空闲态候选框选。
    void beginIdleWindowSelection(const QPoint& screenPos);

    // 更新空闲态候选框选。
    void updateIdleWindowSelection(const QPoint& screenPos);

    // 结束空闲态候选框选。
    bool finishIdleWindowSelection(const QPoint& screenPos);

    // 重置所有子模式
    void resetSubModes();

    // 准备图元子模式（进入绘图状态）
    void preparePrimitiveSubMode();

    // 处理命令状态下的鼠标左键按下
    // @param worldPos 世界坐标位置
    void handleLeftPressInCommand(const QVector3D& worldPos);

    // 检查是否有多段线命令处于活动状态
    // @return 如果多段线命令活动则返回 true
    bool isPolylineCommandActive() const;

    // 设置多段线输入模式
    // @param useArc 是否使用圆弧模式
    // @return 如果成功设置模式返回 true
    bool setPolylineInputMode(bool useArc);

    // 将屏幕坐标转换为当前世界坐标
    // @param screenPos 屏幕坐标
    // @return 对应的世界坐标
    QVector3D currentWorldPos(const QPoint& screenPos) const;

    // 尝试把 currentPos 与当前光标位置同步，避免键盘确认后预览滞后。
    void syncCurrentPosWithCursor();

    // 当前命令阶段是否正在等待输入一个点。
    bool isAwaitingPointInput() const;

    // 对活动命令执行一次“确认”动作（Enter/右键）。
    bool confirmActiveCommand();

    // 按当前命令状态提交一个点（与左键提交共享路径）。
    bool commitCommandPoint(const QVector3D& worldPos);

    // 尝试从动态输入缓冲解析并提交一个点。
    bool submitDynamicInputBuffer();

    // 在当前命令上下文中解析动态输入字符串。
    bool tryResolveDynamicInputPoint(const QString& inputText, QVector3D& worldPoint, QString& errorMessage) const;

    // 获取动态输入参考点（相对坐标/极坐标的基准点）。
    QVector3D dynamicInputReferencePoint() const;

    // 对输入点应用正交约束。
    QVector3D applyOrthoConstraint(const QVector3D& worldPos) const;

    // 在提示栏追加动态输入状态说明。
    QString appendDynamicInputPromptState(const QString& basePrompt) const;

    // 当前“点参数输入”阶段键。
    QString currentPointInputStageKey() const;

    // 同步点参数动态输入会话（阶段切换时自动重置）。
    void syncPointDynamicInputSession();

    // 重置点参数动态输入会话。
    void resetPointDynamicInputSession(const QString& stageKey = QString());

    // 当前是否处于点参数动态输入（字段模式）。
    bool isPointDynamicFieldModeActive() const;

    // 当前是否存在任何键盘动态输入内容。
    bool hasPendingDynamicKeyboardInput() const;

    // 应用字段模式动态输入覆盖（用于预览与提交）。
    QVector3D applyPointDynamicFieldOverride(const QVector3D& worldPos, bool includeEditingValue) const;

    // 尝试解析字段缓冲为数值。
    bool tryParseDynamicFieldBuffer(double& value, QString& errorMessage) const;

    // 提交当前字段编辑值；无编辑值时按当前坐标锁定字段。
    bool commitActiveDynamicField(QString& errorMessage);

    // 处理字段切换（Tab / Shift+Tab）。
    bool handleDynamicFieldTab(int step);

    // 提交点参数动态输入（字段模式）。
    bool submitPointDynamicFieldInput();

    // 格式化动态输入数值显示。
    static QString formatDynamicInputValue(double value);

    // 是否处于动态命令输入态（空闲态键盘命令匹配）。
    bool isDynamicCommandModeActive() const;

    // 清空动态命令输入态。
    void clearDynamicCommandMode();

    // 获取动态命令匹配索引集合。
    QVector<int> collectDynamicCommandMatchIndices() const;

    // 规范化动态命令候选索引。
    void normalizeDynamicCommandSelectionIndex();

    // 切换动态命令候选项。
    bool cycleDynamicCommandSelection(int step);

    // 执行当前选中的动态命令。
    bool executeSelectedDynamicCommand();

    // 按规范命令名执行空闲态命令。
    bool executeIdleCommandByCanonical(const QString& canonicalCommand);

    // 删除当前选中图元。
    bool deleteSelectedEntity();

    // 修改当前选中图元颜色。
    bool changeSelectedEntityColor();

private:
    // 关联的 CAD 视图对象
    CadViewer* m_viewer = nullptr;

    // 关联的 CAD 编辑器对象
    CadEditer* m_editer = nullptr;

    // 绘图状态机，管理绘图状态和流程
    DrawStateMachine m_drawState;

    // 空闲态窗口框选状态。
    bool m_idleWindowSelectionTracking = false;
    bool m_idleWindowSelectionDragging = false;
    QPoint m_idleWindowSelectionAnchor;
    QPoint m_idleWindowSelectionCurrent;
};
