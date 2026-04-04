#pragma once

// ── Qt OpenGL ────────────────────────────────────────────────────
#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_5_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>

// ── Qt 基础 ──────────────────────────────────────────────────────
#include <QMatrix4x4>
#include <QVector3D>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QPoint>
#include <QColor>
#include <QRect>
#include <QElapsedTimer>

// ── 标准库 ───────────────────────────────────────────────────────
#include <vector>
#include <unordered_map>
#include <memory>

// ── 前向声明
// 数据层
class DfxDocument;

class IEntity;
// 物体ID
using EntityId = uint64_t;


// OrbitalCamera — 纯正交轨道相机，仿 AutoCAD 视图模型
struct OrbitalCamera
{
    // ── 轨道参数 ─────────────────────────────────────────────────
    QVector3D target = { 0.0f, 0.0f, 0.0f };  // 注视点（轨道圆心）
    float     distance = 500.0f;                  // 相机到注视点的距离
    float     azimuth = 0.0f;                    // 水平旋转角（°），绕 Y 轴
    float     elevation = 90.0f;                   // 垂直仰角（°），90°= 俯视
    // ── 正交缩放 ─────────────────────────────────────────────────
    //  viewHeight = 视口在世界空间中的可见高度（单位：世界坐标）
    //  可见宽度 = viewHeight × aspectRatio
    //  放大 → viewHeight 减小；缩小 → viewHeight 增大
    float     viewHeight = 200.0f;
    // ── 裁剪面 ───────────────────────────────────────────────────
    float     nearPlane = 0.1f;
    float     farPlane = 100000.0f;
    // ── 限制常量 ─────────────────────────────────────────────────
    static constexpr float kMinViewHeight = 0.001f;
    static constexpr float kMaxViewHeight = 1e7f;
    static constexpr float kMinElevation = -89.9f;
    static constexpr float kMaxElevation = 89.9f;
    // ─────────────────────────────────────────────────────────────
    //  计算相机世界位置
    //  eye = target + distance × spherical(azimuth, elevation)
    // ─────────────────────────────────────────────────────────────
    QVector3D eyePosition() const;
    // ─────────────────────────────────────────────────────────────
    //  相机局部坐标轴（均为单位向量）
    //  forward : 指向 target 的方向（eye → target）
    //  right   : 相机右方向
    //  up      : 相机上方向
    // ─────────────────────────────────────────────────────────────
    QVector3D forwardDirection() const;
    QVector3D rightDirection()   const;
    QVector3D upDirection()      const;
    // ─────────────────────────────────────────────────────────────
    //  构建矩阵
    // ─────────────────────────────────────────────────────────────
    QMatrix4x4 viewMatrix()                            const;
    QMatrix4x4 projectionMatrix(float aspectRatio)     const;  // 纯正交
    QMatrix4x4 viewProjectionMatrix(float aspectRatio) const;
    // ─────────────────────────────────────────────────────────────
    //  视角操作
    // ─────────────────────────────────────────────────────────────
    void orbit(float deltaAzimuth, float deltaElevation);  // 旋转（度）
    void pan(float worldDx, float worldDy);                // 沿相机平面平移
    void zoom(float factor);                               // 缩放：factor > 1 放大
    void zoomAtPoint(float factor,                         // 以指定世界点为中心缩放
        const QVector3D& worldAnchor,
        float aspectRatio);


    // ── FitAll：自动调整使场景包围盒充满视口 ────────────────────
    void fitAll(const QVector3D& sceneMin,
        const QVector3D& sceneMax,
        float            aspectRatio);
};

// ═════════════════════════════════════════════════════════════════
//
//  GPU 侧缓冲区：每个实体对应一份 GPU 缓冲，
//  场景更新时增量刷新，避免每帧重建全量数据
//
// ═════════════════════════════════════════════════════════════════
struct EntityGpuBuffer {
    QOpenGLBuffer              vbo{ QOpenGLBuffer::VertexBuffer };
    QOpenGLVertexArrayObject   vao;
    int    vertexCount = 0;
    GLenum primitiveType = GL_LINE_STRIP;   // GL_LINES / GL_LINE_STRIP / GL_POINTS
    QVector3D color = { 1.0f, 1.0f, 1.0f };  // 实体颜色（来自图层/实体属性）
};

// ═════════════════════════════════════════════════════════════════
//
//  鼠标交互状态机
//
// ═════════════════════════════════════════════════════════════════
enum class ViewInteractionMode 
{
    Idle,             // 空闲，等待交互
    Orbiting,         // 旋转视角（Shift + 中键 拖拽）
    Panning,          // 平移视角（中键 拖拽）
    ZoomWindow,       // 框选缩放（待实现）
    SelectionBox,     // 框选实体（左键拖拽）
};


// Dxf视图层
class DxfViewer : public QOpenGLWidget, QOpenGLFunctions_4_5_Core
{
    Q_OBJECT

public:
    // 构造函数
    explicit DxfViewer(QWidget* parent = nullptr);
    // 析构函数
    ~DxfViewer() override;

    
    // 缩放
    void  zoomIn(float factor = 1.1f);
    void  zoomOut(float factor = 1.1f);

    // 坐标转换工具
    QVector3D screenToWorld(const QPoint& screenPos, float depth = 0.0f) const;
    QPoint    worldToScreen(const QVector3D& worldPos) const;

protected:
    // OpenGL 核心虚函数
    void initializeGL()         override;
    void resizeGL(int w, int h) override;
    void paintGL()              override;

private:
    // ═════════════════════════════════════════════════════════════
    //  初始化子系统
    // ═════════════════════════════════════════════════════════════
    void initShaders();        // 编译链接 GLSL 着色器
    void initGridBuffer();     // 构建辅助网格 GPU 数据
    void initAxisBuffer();     // 构建坐标轴 GPU 数据
    void initSelectionBoxBuffer();  // 构建框选矩形 GPU 数据

    // ═════════════════════════════════════════════════════════════
    //  GPU 缓冲管理（增量更新，非全量重建）
    //  从实体接口获取渲染数据，上传到 GPU，写入 m_entityBuffers
    // ═════════════════════════════════════════════════════════════
    void uploadEntity(const IEntity* entity);
    void removeEntityBuffer(EntityId id);
    void rebuildAllBuffers();          // 全量重建（文档重置时调用）
    void clearAllBuffers();            // 释放所有 GPU 资源

    // ═════════════════════════════════════════════════════════════
    //  渲染子程序（paintGL 内部分步调用）
    // ═════════════════════════════════════════════════════════════
    void renderGrid();                 // 渲染辅助网格
    void renderAxis();                 // 渲染坐标轴指示器
    void renderEntities();             // 遍历场景，渲染所有可见实体

    // ═════════════════════════════════════════════════════════════
    //  计算辅助
    // ═════════════════════════════════════════════════════════════
    float aspectRatio() const;                   // width / height
    float pixelToWorldScale() const;             // 当前缩放下 1px 对应世界单位

private:

    // 数据层指针，方便数据的访问
    DfxDocument* m_scene = nullptr;

    // 相机
    OrbitalCamera  m_camera;

    // ─────────────────────────────────────────────────────────────
    //  着色器程序
    // ─────────────────────────────────────────────────────────────
    std::unique_ptr<QOpenGLShaderProgram> m_entityShader;   // 实体渲染着色器
    std::unique_ptr<QOpenGLShaderProgram> m_gridShader;     // 网格渲染着色器

    // ─────────────────────────────────────────────────────────────
    //  实体 GPU 缓冲表
    //  key = EntityId, value = 该实体在 GPU 上的 VBO/VAO
    //  场景更新时增量维护，避免每帧重传所有顶点
    // ─────────────────────────────────────────────────────────────
    std::unordered_map<EntityId, EntityGpuBuffer> m_entityBuffers;

    // ─────────────────────────────────────────────────────────────
    //  辅助渲染对象（网格 + 坐标轴）
    // ─────────────────────────────────────────────────────────────
    QOpenGLBuffer              m_gridVbo{ QOpenGLBuffer::VertexBuffer };
    QOpenGLVertexArrayObject   m_gridVao;
    int                        m_gridVertexCount = 0;

    QOpenGLBuffer              m_axisVbo{ QOpenGLBuffer::VertexBuffer };
    QOpenGLVertexArrayObject   m_axisVao;
    int                        m_axisVertexCount = 0;

    // ─────────────────────────────────────────────────────────────
    int m_viewportWidth = 1;
    int m_viewportHeight = 1;

    // ─────────────────────────────────────────────────────────────
    //  OpenGL 初始化标记
    // ─────────────────────────────────────────────────────────────
    bool m_glInitialized = false;
};