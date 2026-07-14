// CadEditer 头文件
// 声明 CadEditer 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 编辑器模块，负责绘图创建、实体修改以及 Undo/Redo 命令栈管理。
#pragma once

#include <memory>
#include <vector>

#include <QColor>
#include <QVector>
#include <QVector3D>

#include "CadProcessVisualUtils.h"
#include "application/process/DocumentProcessState.h"

class CadDocument;
class CadItem;
class DRW_Entity;
class DrawStateMachine;

// 文档编辑器：
// - 接收 Controller 发起的编辑/绘图命令
// - 依据 DrawStateMachine 当前子状态完成图元创建
// - 通过命令栈实现 undo / redo
class CadEditer
{
public:
    CadEditer() = default;

    // 析构编辑器对象
    ~CadEditer();

    // 绑定当前编辑目标文档
    // @param document 文档对象指针
    void setDocument(CadDocument* document);
    void setProcessState(cadcam::process::DocumentProcessState* processState);

    // 清空 Undo / Redo 历史
    void clearHistory();

    // 取消当前 transient 编辑命令
    void cancelTransientCommand();

    // 查询是否可以撤销
    // @return 如果撤销栈非空返回 true，否则返回 false
    bool canUndo() const;

    // 查询是否可以重做
    // @return 如果重做栈非空返回 true，否则返回 false
    bool canRedo() const;

    // 执行撤销
    // @return 如果撤销成功返回 true，否则返回 false
    bool undo();

    // 执行重做
    // @return 如果重做成功返回 true，否则返回 false
    bool redo();

    // 处理左键点击驱动的绘图或编辑逻辑
    // @param previousState 点击前的状态机快照
    // @param currentState 点击后可修改的状态机
    // @param worldPos 当前点击对应的世界坐标
    // @return 如果事件被编辑器消费返回 true，否则返回 false
    bool handleLeftPress(const DrawStateMachine& previousState, DrawStateMachine& currentState, const QVector3D& worldPos);

    // 结束当前活动多段线命令
    // @param drawState 当前绘图状态机
    // @param closePolyline 是否闭合多段线
    // @return 如果成功生成实体返回 true，否则返回 false
    bool finishActivePolyline(DrawStateMachine& drawState, bool closePolyline);

    // 开始移动编辑命令
    // @param drawState 当前绘图状态机
    // @param item 待移动实体
    // @return 如果命令成功进入活动状态返回 true，否则返回 false
    bool beginMove(DrawStateMachine& drawState, CadItem* item);

    // 开始移动编辑命令（多实体）
    bool beginMove(DrawStateMachine& drawState, const QVector<CadItem*>& items);

    // 开始控制点编辑命令
    bool beginGripEdit(DrawStateMachine& drawState, CadItem* item, const CadSelectionHandleInfo& handle);

    // 删除指定实体
    // @param item 待删除实体
    // @return 如果删除成功返回 true，否则返回 false
    bool deleteEntity(CadItem* item);

    // 批量删除指定实体
    bool deleteEntities(const QVector<CadItem*>& items);

    // 复制指定实体，并按偏移量生成一个副本
    bool copyEntity(CadItem* item, const QVector3D& delta);

    // 批量复制指定实体，并按偏移量生成副本
    bool copyEntities(const QVector<CadItem*>& items, const QVector3D& delta);

    // 围绕基点旋转指定实体，角度单位为度
    bool rotateEntity(CadItem* item, const QVector3D& basePoint, double angleDegrees);

    // 围绕基点批量旋转指定实体，角度单位为度
    bool rotateEntities(const QVector<CadItem*>& items, const QVector3D& basePoint, double angleDegrees);

    // 围绕基点按统一倍率缩放指定实体
    bool scaleEntity(CadItem* item, const QVector3D& basePoint, double scaleFactor);

    // 围绕基点按统一倍率批量缩放指定实体
    bool scaleEntities(const QVector<CadItem*>& items, const QVector3D& basePoint, double scaleFactor);

    // 对指定实体执行矩形阵列
    bool arrayEntity(CadItem* item, int rowCount, int columnCount, const QVector3D& rowOffset, const QVector3D& columnOffset);

    // 对指定实体集合执行矩形阵列
    bool rectangularArrayEntities(const QVector<CadItem*>& items, int rowCount, int columnCount, const QVector3D& rowOffset, const QVector3D& columnOffset);

    // 对指定实体集合执行镜像；eraseSource=true 时删除原图元。
    bool mirrorEntities(const QVector<CadItem*>& items, const QVector3D& firstPoint, const QVector3D& secondPoint, bool eraseSource);

    // 对指定实体集合执行环形阵列。
    bool polarArrayEntities(const QVector<CadItem*>& items, const QVector3D& center, int itemCount, double totalAngleDegrees, bool rotateItems);

    // 偏移一个实体，返回是否成功。
    bool offsetEntity(CadItem* item, double distance);

    // 按边界修剪或延伸目标实体。
    bool trimEntity(CadItem* boundaryItem, CadItem* targetItem, bool trimStart);
    bool extendEntity(CadItem* boundaryItem, CadItem* targetItem, bool extendStart);

    // 合并选中线/直线型多段线为一条轻量多段线。
    bool joinEntities(const QVector<CadItem*>& items);

    // 两条直线执行圆角或直角(倒角)。
    bool filletEntities(CadItem* firstItem, CadItem* secondItem, double radius);
    bool chamferEntities(CadItem* firstItem, CadItem* secondItem, double firstDistance, double secondDistance);

    // 修改指定实体颜色
    // @param item 目标实体
    // @param color 新颜色
    // @param colorIndex 可选 ACI 颜色索引，小于 0 时使用 true color
    // @return 如果修改成功返回 true，否则返回 false
    bool changeEntityColor(CadItem* item, const QColor& color, int colorIndex = -1);

    // 修改指定实体图层。
    // @param item 目标实体
    // @param layerName 新图层名
    // @return 如果修改成功返回 true，否则返回 false
    bool changeEntityLayer(CadItem* item, const QString& layerName);

    // 编辑命令抽象基类：
    // 封装一次可执行且可撤销的文档修改操作。
    class EditCommand
    {
    public:
        // 析构命令对象
        virtual ~EditCommand() = default;

        // 执行命令
        // @return 如果执行成功返回 true，否则返回 false
        virtual bool execute() = 0;

        // 撤销命令
        // @return 如果撤销成功返回 true，否则返回 false
        virtual bool undo() = 0;
    };

private:
    // 执行命令并压入 Undo 栈
    // @param command 待执行的命令对象
    // @return 如果执行成功返回 true，否则返回 false
    bool executeCommand(std::unique_ptr<EditCommand> command);

    // 处理点绘制命令
    bool handlePointDrawing(const DrawStateMachine& previousState, DrawStateMachine& currentState, const QVector3D& worldPos);

    // 处理直线绘制命令
    bool handleLineDrawing(const DrawStateMachine& previousState, DrawStateMachine& currentState, const QVector3D& worldPos);

    // 处理构造线绘制命令
    bool handleXlineDrawing(const DrawStateMachine& previousState, DrawStateMachine& currentState, const QVector3D& worldPos);

    // 处理矩形绘制命令
    bool handleRectangleDrawing(const DrawStateMachine& previousState, DrawStateMachine& currentState, const QVector3D& worldPos);

    // 处理多边形绘制命令
    bool handlePolygonDrawing(const DrawStateMachine& previousState, DrawStateMachine& currentState, const QVector3D& worldPos);

    // 处理圆绘制命令
    bool handleCircleDrawing(const DrawStateMachine& previousState, DrawStateMachine& currentState, const QVector3D& worldPos);

    // 处理圆弧绘制命令
    bool handleArcDrawing(const DrawStateMachine& previousState, DrawStateMachine& currentState, const QVector3D& worldPos);

    // 处理椭圆绘制命令
    bool handleEllipseDrawing(const DrawStateMachine& previousState, DrawStateMachine& currentState, const QVector3D& worldPos);

    // 处理多段线/轻量多段线绘制命令
    // @param lightweight 是否创建轻量多段线
    bool handlePolylineDrawing(const DrawStateMachine& previousState, DrawStateMachine& currentState, const QVector3D& worldPos, bool lightweight);

    // 处理移动编辑命令
    bool handleMoveEditing(const DrawStateMachine& previousState, DrawStateMachine& currentState, const QVector3D& worldPos);

    // 处理控制点编辑命令
    bool handleGripEditing(const DrawStateMachine& previousState, DrawStateMachine& currentState, const QVector3D& worldPos);

    // 向文档追加新实体
    // @param entity 待追加的原生 DXF 实体
    // @return 如果追加成功返回 true，否则返回 false
    bool addEntity(std::unique_ptr<DRW_Entity> entity);

private:
    // 当前绑定的文档对象
    CadDocument* m_document = nullptr;
    cadcam::process::DocumentProcessState* m_processState = nullptr;

    // Move 命令当前锁定的目标实体
    CadItem* m_moveTarget = nullptr;

    // Move 命令当前锁定的目标实体集合（用于多选移动）
    QVector<CadItem*> m_moveTargets;

    // 控制点编辑当前锁定的目标实体
    CadItem* m_gripTarget = nullptr;

    // 控制点编辑当前锁定的控制点索引
    int m_gripPointIndex = -1;

    // Undo 栈
    std::vector<std::unique_ptr<EditCommand>> m_undoStack;

    // Redo 栈
    std::vector<std::unique_ptr<EditCommand>> m_redoStack;
};
