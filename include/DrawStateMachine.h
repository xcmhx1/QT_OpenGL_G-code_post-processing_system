#pragma once

#include <QColor>
#include <QPoint>
#include <QVector3D>

// 定义绘图图元类型的枚举
enum class DrawPrimitiveKind
{
    None,     // 无图元
    Point,    // 点
    Line,     // 直线
    Circle,   // 圆
    Arc,      // 弧
    Ellipse,  // 椭圆
    Polyline, // 多段线
    LWPolyline// 轻型多段线
};

// 定义点绘图子模式的枚举
enum class PointDrawSubMode
{
    Idle,          // 空闲状态
    AwaitPosition, // 等待用户指定点的位置
};

// 定义直线绘图子模式的枚举
enum class LineDrawSubMode
{
    Idle,          // 空闲状态
    AwaitStartPoint, // 等待用户指定线的起点
    AwaitEndPoint,   // 等待用户指定线的终点
};

// 定义圆绘图子模式的枚举
enum class CircleDrawSubMode
{
    Idle,        // 空闲状态
    AwaitCenter, // 等待用户指定圆心
    AwaitRadius, // 等待用户指定半径
};

// 定义弧绘图子模式的枚举
enum class ArcDrawSubMode
{
    Idle,          // 空闲状态
    AwaitCenter,   // 等待用户指定圆心
    AwaitRadius,   // 等待用户指定半径
    AwaitStartAngle, // 等待用户指定弧的启始角度
    AwaitEndAngle,   // 等待用户指定弧的终点角度
};

// 定义椭圆绘图子模式的枚举
enum class EllipseDrawSubMode
{
    Idle,           // 空闲状态
    AwaitCenter,    // 等待用户指定椭圆中心
    AwaitMajorAxis, // 等待用户指定长轴（方向和长度）
    AwaitMinorAxis, // 等待用户指定短轴（方向和长度）
};

// 定义折线绘图子模式的枚举
enum class PolylineDrawSubMode
{
    Idle,                 // 空闲状态
    AwaitFirstPoint,      // 等待用户指定第一个点
    AwaitLineEndPoint,    // 等待用户指定直线段的终点
    AwaitArcEndPoint,     // 等待用户指定圆弧段的终点
};

// 定义轻量多段线绘图子模式的枚举
enum class LWPolylineDrawSubMode
{
    Idle,                 // 空闲状态
    AwaitFirstPoint,      // 等待用户指定第一个点
    AwaitLineEndPoint,    // 等待用户指定直线段的终点
    AwaitArcEndPoint,     // 等待用户指定圆弧段的终点
};

// 统一维护绘图/编辑阶段会复用的状态和鼠标上下文。
// 这是一个状态机类，用于管理用户在绘图或编辑时的各种交互状态。
class DrawStateMachine
{
public:
    // 重置所有状态到默认值
    void reset();

    // 返回全局唯一的实例，方便其他类的访问
    static DrawStateMachine& instance()
    {
        return s_instance; // 直接返回已创建的静态实例
    }

public:
    // 是否正在进行绘图操作
    bool isDrawing = false;
    // 当前正在绘制的图元类型
    DrawPrimitiveKind drawType = DrawPrimitiveKind::None;
    // 当前绘图的颜色，默认为白色
    QColor drawingColor = QColor(255, 255, 255);

    // 各图元自己的子状态机，当前由 drawingPrimitiveKind 决定使用哪一个。
    // 这些成员变量用于存储特定图元绘制过程中的子状态。
    // 点状态机
    PointDrawSubMode pointSubMode = PointDrawSubMode::Idle;
    // 直线状态机
    LineDrawSubMode lineSubMode = LineDrawSubMode::Idle;
    // 圆状态机
    CircleDrawSubMode circleSubMode = CircleDrawSubMode::Idle;
    // 圆弧状态机
    ArcDrawSubMode arcSubMode = ArcDrawSubMode::Idle;
    // 椭圆状态机
    EllipseDrawSubMode ellipseSubMode = EllipseDrawSubMode::Idle;
    // 多段线状态机 
    PolylineDrawSubMode polylineSubMode = PolylineDrawSubMode::Idle;
    // 轻量多段线状态机
    LWPolylineDrawSubMode lwPolylineSubMode = LWPolylineDrawSubMode::Idle;

    // 鼠标上一次的世界坐标（3D）
    QVector3D lastPos;
    // 鼠标当前的世界坐标（3D）
    QVector3D currentPos;
    // 当前被按下的鼠标按键（单个）
    Qt::MouseButton activeButton = Qt::NoButton;
    // 当前被按下的所有鼠标按键（组合）
    Qt::MouseButtons pressedButtons = Qt::NoButton;
    // 当前按下的键盘修饰符（如 Shift, Ctrl, Alt）
    Qt::KeyboardModifiers keyboardModifiers = Qt::NoModifier;

    // 静态成员变量，在程序启动时即被初始化
    static DrawStateMachine s_instance;
};