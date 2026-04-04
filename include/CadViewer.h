#pragma once

#include <memory>
#include <unordered_map>

#include <QColor>
#include <QKeyEvent>
#include <QMatrix4x4>
#include <QMouseEvent>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions_4_5_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPoint>
#include <QVector3D>
#include <QWheelEvent>

class CadItem;
class CadDocument;

using EntityId = quintptr;

struct OrbitalCamera
{
    QVector3D target = { 0.0f, 0.0f, 0.0f };
    float distance = 500.0f;
    float azimuth = -90.0f;
    float elevation = 89.9f;
    float viewHeight = 200.0f;
    float nearPlane = -100000.0f;
    float farPlane = 100000.0f;

    static constexpr float kMinViewHeight = 0.001f;
    static constexpr float kMaxViewHeight = 1.0e7f;
    static constexpr float kMinElevation = -89.9f;
    static constexpr float kMaxElevation = 89.9f;

    QVector3D eyePosition() const;
    QVector3D forwardDirection() const;
    QVector3D rightDirection() const;
    QVector3D upDirection() const;

    QMatrix4x4 viewMatrix() const;
    QMatrix4x4 projectionMatrix(float aspectRatio) const;
    QMatrix4x4 viewProjectionMatrix(float aspectRatio) const;

    void orbit(float deltaAzimuth, float deltaElevation);
    void pan(float worldDx, float worldDy);
    void zoom(float factor);
    void zoomAtPoint(float factor, const QVector3D& worldAnchor, float aspectRatio);
    void fitAll(const QVector3D& sceneMin, const QVector3D& sceneMax, float aspectRatio);
};

struct EntityGpuBuffer
{
    QOpenGLBuffer vbo{ QOpenGLBuffer::VertexBuffer };
    QOpenGLVertexArrayObject vao;
    int vertexCount = 0;
    GLenum primitiveType = GL_LINE_STRIP;
    QVector3D color = { 1.0f, 1.0f, 1.0f };
};

enum class ViewInteractionMode
{
    Idle,
    Orbiting,
    Panning,
    SelectionBox,
};

class CadViewer : public QOpenGLWidget, protected QOpenGLFunctions_4_5_Core
{
    Q_OBJECT

public:
    explicit CadViewer(QWidget* parent = nullptr);
    ~CadViewer() override;

    void setDocument(CadDocument* document);
    void fitScene();
    void zoomIn(float factor = 1.1f);
    void zoomOut(float factor = 1.1f);

    QVector3D screenToWorld(const QPoint& screenPos, float depth = 0.0f) const;
    QPoint worldToScreen(const QVector3D& worldPos) const;

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;

private:
    void initShaders();
    void initGridBuffer();
    void initAxisBuffer();
    void initSelectionBoxBuffer();

    void uploadEntity(const CadItem* entity);
    void removeEntityBuffer(EntityId id);
    void rebuildAllBuffers();
    void clearAllBuffers();

    void renderGrid();
    void renderAxis();
    void renderEntities();
    void renderOrbitMarker();

    void updateSceneBounds();
    void orbitCameraAroundSceneCenter(float deltaAzimuth, float deltaElevation);
    EntityId pickEntity(const QPoint& screenPos) const;

    float aspectRatio() const;
    float pixelToWorldScale() const;

private:
    CadDocument* m_scene = nullptr;
    OrbitalCamera m_camera;

    std::unique_ptr<QOpenGLShaderProgram> m_entityShader;
    std::unique_ptr<QOpenGLShaderProgram> m_gridShader;

    std::unordered_map<EntityId, EntityGpuBuffer> m_entityBuffers;

    QOpenGLBuffer m_gridVbo{ QOpenGLBuffer::VertexBuffer };
    QOpenGLVertexArrayObject m_gridVao;
    int m_gridVertexCount = 0;

    QOpenGLBuffer m_axisVbo{ QOpenGLBuffer::VertexBuffer };
    QOpenGLVertexArrayObject m_axisVao;
    int m_axisVertexCount = 0;

    QOpenGLBuffer m_orbitMarkerVbo{ QOpenGLBuffer::VertexBuffer };
    QOpenGLVertexArrayObject m_orbitMarkerVao;

    int m_viewportWidth = 1;
    int m_viewportHeight = 1;
    bool m_glInitialized = false;
    bool m_buffersDirty = true;
    bool m_hasSceneBounds = false;

    QVector3D m_sceneMin;
    QVector3D m_sceneMax;
    QVector3D m_orbitCenter;

    ViewInteractionMode m_interactionMode = ViewInteractionMode::Idle;
    QPoint m_lastMousePos;
    EntityId m_selectedEntityId = 0;
};
