// CadController 头文件
// 声明 CadController 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 输入控制模块，负责解释键盘、鼠标、滚轮事件并驱动绘图/编辑命令。

#pragma once

// Qt 核心模块
#include <QColor>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QString>
#include <QVector3D>
#include <QWheelEvent>

// CAD 模块内部依赖
#include "DrawStateMachine.h"

// 前向声明
class CadViewer;
class CadEditer;

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

private:
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

private:
    // 关联的 CAD 视图对象
    CadViewer* m_viewer = nullptr;

    // 关联的 CAD 编辑器对象
    CadEditer* m_editer = nullptr;

    // 绘图状态机，管理绘图状态和流程
    DrawStateMachine m_drawState;
};
