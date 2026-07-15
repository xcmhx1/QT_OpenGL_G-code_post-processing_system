#include "platform/pch.h" // 假设这是预编译头文件，如果不是，请根据实际情况移除或修改

#include "cad/editing/DrawStateMachine.h" // 包含 DrawStateMachine 的头文件

// 饿汉式单例的静态成员变量初始化
// 在程序启动时，这个唯一的 DrawStateMachine 实例就会被创建。
DrawStateMachine DrawStateMachine::s_instance;

// 实现 reset 方法，将所有成员变量重置为它们的默认值
void DrawStateMachine::reset()
{
    const QColor preservedDrawingColor = drawingColor;
    const QString preservedLayerName = drawingLayerName;
    const int preservedColorIndex = drawingColorIndex;
    const bool preservedOrthoEnabled = orthoEnabled;
    const bool preservedPolarTrackingEnabled = polarTrackingEnabled;
    const int preservedPolygonSideCount = polygonSideCount;
    const bool preservedPolygonCircumscribedAboutCircle = polygonCircumscribedAboutCircle;

    // 重置基本状态
    isDrawing = false;
    drawType = DrawType::None; 
    drawingColor = preservedDrawingColor.isValid() ? preservedDrawingColor : QColor(255, 255, 255);
    drawingLayerName = preservedLayerName.trimmed().isEmpty() ? QStringLiteral("0") : preservedLayerName;
    drawingColorIndex = preservedColorIndex;
    editType = EditType::None;
    moveSubMode = MoveEditSubMode::Idle;
    gripSubMode = GripEditSubMode::Idle;
    gripPointIndex = -1;
    commandPoints.clear();
    commandBulges.clear();
    polylineArcMode = false;
    lwPolylineArcMode = false;

    // 重置所有图元的子状态机为 Idle
    pointSubMode = PointDrawSubMode::Idle;
    lineSubMode = LineDrawSubMode::Idle;
    circleSubMode = CircleDrawSubMode::Idle;
    rectangleSubMode = RectangleDrawSubMode::Idle;
    polygonSubMode = PolygonDrawSubMode::Idle;
    arcSubMode = ArcDrawSubMode::Idle;
    ellipseSubMode = EllipseDrawSubMode::Idle;
    polylineSubMode = PolylineDrawSubMode::Idle;
    lwPolylineSubMode = LWPolylineDrawSubMode::Idle; 
    rotatePreviewActive = false;
    rotatePreviewBasePoint = QVector3D();
    rotatePreviewAngleDegrees = 0.0;
    copyPreviewActive = false;
    copyPreviewBasePoint = QVector3D();
    copyPreviewDelta = QVector3D();
    mirrorPreviewActive = false;
    mirrorPreviewFirstPoint = QVector3D();
    mirrorPreviewSecondPoint = QVector3D();
    rectangularArrayPreviewActive = false;
    rectangularArrayPreviewRows = 1;
    rectangularArrayPreviewColumns = 1;
    rectangularArrayPreviewRowOffset = QVector3D();
    rectangularArrayPreviewColumnOffset = QVector3D();
    circularArrayPreviewActive = false;
    circularArrayPreviewCenter = QVector3D();
    circularArrayPreviewCount = 1;
    circularArrayPreviewTotalAngleDegrees = 0.0;
    circularArrayPreviewRotateItems = true;
    offsetPreviewActive = false;
    offsetPreviewDistance = 0.0;
    scalePreviewActive = false;
    scalePreviewBasePoint = QVector3D();
    scalePreviewFactor = 1.0;

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
    dynamicInputBuffer.clear();
    dynamicInputStageKey.clear();
    dynamicInputExpressionMode = false;
    dynamicInputActiveFieldIndex = 0;
    dynamicInputFieldBuffer.clear();
    dynamicInputXLocked = false;
    dynamicInputYLocked = false;
    dynamicInputXValue = 0.0;
    dynamicInputYValue = 0.0;
    dynamicCommandBuffer.clear();
    dynamicCommandActiveIndex = 0;
    orthoEnabled = preservedOrthoEnabled;
    polarTrackingEnabled = preservedPolarTrackingEnabled;
    polygonSideCount = preservedPolygonSideCount;
    polygonCircumscribedAboutCircle = preservedPolygonCircumscribedAboutCircle;
}

bool DrawStateMachine::hasActiveCommand() const
{
    return isDrawing || editType != EditType::None;
}

