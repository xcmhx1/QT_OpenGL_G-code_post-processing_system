#include "pch.h" // 假设这是预编译头文件，如果不是，请根据实际情况移除或修改

#include "DrawStateMachine.h" // 包含 DrawStateMachine 的头文件

// 饿汉式单例的静态成员变量初始化
// 在程序启动时，这个唯一的 DrawStateMachine 实例就会被创建。
DrawStateMachine DrawStateMachine::s_instance;

// 实现 reset 方法，将所有成员变量重置为它们的默认值
void DrawStateMachine::reset()
{
    // 重置基本状态
    isDrawing = false;
    drawType = DrawType::None; 
    drawingColor = QColor(255, 255, 255); 
    editType = EditType::None;
    moveSubMode = MoveEditSubMode::Idle;
    commandPoints.clear();

    // 重置所有图元的子状态机为 Idle
    pointSubMode = PointDrawSubMode::Idle;
    lineSubMode = LineDrawSubMode::Idle;
    circleSubMode = CircleDrawSubMode::Idle;
    arcSubMode = ArcDrawSubMode::Idle;
    ellipseSubMode = EllipseDrawSubMode::Idle;
    polylineSubMode = PolylineDrawSubMode::Idle;
    lwPolylineSubMode = LWPolylineDrawSubMode::Idle; 

    // 重置鼠标相关的世界坐标位置
    pressScreenPos = QPoint();
    lastScreenPos = QPoint();
    currentScreenPos = QPoint();
    lastPos = QVector3D();     
    currentPos = QVector3D();

    // 重置鼠标按钮和键盘修饰符
    activeButton = Qt::NoButton;
    pressedButtons = Qt::NoButton;
    keyboardModifiers = Qt::NoModifier;
}

bool DrawStateMachine::hasActiveCommand() const
{
    return isDrawing || editType != EditType::None;
}
