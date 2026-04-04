#include "pch.h"

#include "CadViewer.h"

#include "CadDocument.h"
#include "CadItem.h"

#include <QKeyEvent>
#include <QOpenGLContext>
#include <QSurfaceFormat>
#include <QVector4D>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kPickThresholdPixels = 10.0f;

EntityId toEntityId(const CadItem* entity)
{
    return static_cast<EntityId>(reinterpret_cast<quintptr>(entity));
}

QPointF projectToScreen
(
    const QVector3D& worldPos,
    const QMatrix4x4& viewProjection,
    int viewportWidth,
    int viewportHeight
)
{
    const QVector4D clip = viewProjection * QVector4D(worldPos, 1.0f);

    if (qFuzzyIsNull(clip.w()))
    {
        return QPointF();
    }

    const QVector3D ndc = clip.toVector3DAffine();

    return QPointF
    (
        (ndc.x() + 1.0f) * 0.5f * viewportWidth,
        (1.0f - ndc.y()) * 0.5f * viewportHeight
    );
}

float distanceToSegmentSquared(const QPointF& point, const QPointF& start, const QPointF& end)
{
    const QPointF segment = end - start;
    const double lengthSquared = segment.x() * segment.x() + segment.y() * segment.y();

    if (lengthSquared <= 1.0e-12)
    {
        const QPointF delta = point - start;
        return static_cast<float>(delta.x() * delta.x() + delta.y() * delta.y());
    }

    const QPointF fromStart = point - start;
    const double t = std::clamp
    (
        (fromStart.x() * segment.x() + fromStart.y() * segment.y()) / lengthSquared,
        0.0,
        1.0
    );

    const QPointF projection = start + segment * t;
    const QPointF delta = point - projection;
    return static_cast<float>(delta.x() * delta.x() + delta.y() * delta.y());
}

GLenum primitiveTypeForEntity(const CadItem* entity)
{
    if (entity == nullptr)
    {
        return GL_LINE_STRIP;
    }

    switch (entity->m_type)
    {
    case DRW::ETYPE::POINT:
        return GL_POINTS;
    case DRW::ETYPE::LINE:
        return GL_LINES;
    default:
        return GL_LINE_STRIP;
    }
}
}

QVector3D OrbitalCamera::eyePosition() const
{
    const float azimuthRad = azimuth * kDegToRad;
    const float elevationRad = elevation * kDegToRad;
    const float cosElevation = std::cos(elevationRad);

    return target + QVector3D
    (
        distance * cosElevation * std::cos(azimuthRad),
        distance * cosElevation * std::sin(azimuthRad),
        distance * std::sin(elevationRad)
    );
}

QVector3D OrbitalCamera::forwardDirection() const
{
    QVector3D forward = target - eyePosition();

    if (qFuzzyIsNull(forward.lengthSquared()))
    {
        return QVector3D(0.0f, 0.0f, -1.0f);
    }

    forward.normalize();
    return forward;
}

QVector3D OrbitalCamera::rightDirection() const
{
    const QVector3D forward = forwardDirection();
    QVector3D referenceUp(0.0f, 0.0f, 1.0f);

    if (std::abs(QVector3D::dotProduct(forward, referenceUp)) > 0.99f)
    {
        referenceUp = QVector3D(0.0f, 1.0f, 0.0f);
    }

    QVector3D right = QVector3D::crossProduct(forward, referenceUp);

    if (qFuzzyIsNull(right.lengthSquared()))
    {
        right = QVector3D(1.0f, 0.0f, 0.0f);
    }
    else
    {
        right.normalize();
    }

    return right;
}

QVector3D OrbitalCamera::upDirection() const
{
    QVector3D up = QVector3D::crossProduct(rightDirection(), forwardDirection());

    if (qFuzzyIsNull(up.lengthSquared()))
    {
        return QVector3D(0.0f, 1.0f, 0.0f);
    }

    up.normalize();
    return up;
}

QMatrix4x4 OrbitalCamera::viewMatrix() const
{
    QMatrix4x4 matrix;
    matrix.lookAt(eyePosition(), target, upDirection());
    return matrix;
}

QMatrix4x4 OrbitalCamera::projectionMatrix(float aspectRatio) const
{
    const float safeAspectRatio = std::max(0.001f, aspectRatio);
    const float halfHeight = viewHeight * 0.5f;
    const float halfWidth = halfHeight * safeAspectRatio;

    QMatrix4x4 matrix;
    matrix.ortho(-halfWidth, halfWidth, -halfHeight, halfHeight, nearPlane, farPlane);
    return matrix;
}

QMatrix4x4 OrbitalCamera::viewProjectionMatrix(float aspectRatio) const
{
    return projectionMatrix(aspectRatio) * viewMatrix();
}

void OrbitalCamera::orbit(float deltaAzimuth, float deltaElevation)
{
    azimuth += deltaAzimuth;
    elevation = std::clamp(elevation + deltaElevation, kMinElevation, kMaxElevation);
}

void OrbitalCamera::pan(float worldDx, float worldDy)
{
    target += rightDirection() * worldDx + upDirection() * worldDy;
}

void OrbitalCamera::zoom(float factor)
{
    if (factor <= 0.0f)
    {
        return;
    }

    viewHeight = std::clamp(viewHeight / factor, kMinViewHeight, kMaxViewHeight);
}

void OrbitalCamera::zoomAtPoint(float factor, const QVector3D& worldAnchor, float aspectRatio)
{
    if (factor <= 0.0f)
    {
        return;
    }

    const QVector3D planeOffset = worldAnchor - target;
    const QVector3D forward = forwardDirection();
    const QVector3D anchorOnCameraPlane = planeOffset - QVector3D::dotProduct(planeOffset, forward) * forward;
    const QVector3D deltaTarget = anchorOnCameraPlane * (1.0f - 1.0f / factor);

    zoom(factor);
    target += deltaTarget;
    Q_UNUSED(aspectRatio);
}

void OrbitalCamera::fitAll(const QVector3D& sceneMin, const QVector3D& sceneMax, float aspectRatio)
{
    const QVector3D size = sceneMax - sceneMin;

    target = (sceneMin + sceneMax) * 0.5f;

    const float safeAspectRatio = std::max(0.001f, aspectRatio);
    const float width = std::max(size.x(), 1.0f);
    const float height = std::max(size.y(), 1.0f);
    const float depth = std::max(size.z(), 1.0f);

    viewHeight = std::clamp(std::max(height, width / safeAspectRatio) * 1.2f, kMinViewHeight, kMaxViewHeight);
    distance = std::max({ width, height, depth, 100.0f }) * 2.0f;
}

CadViewer::CadViewer(QWidget* parent)
    : QOpenGLWidget(parent)
{
    QSurfaceFormat surfaceFormat = format();
    surfaceFormat.setRenderableType(QSurfaceFormat::OpenGL);
    surfaceFormat.setVersion(4, 5);
    surfaceFormat.setProfile(QSurfaceFormat::CoreProfile);
    surfaceFormat.setDepthBufferSize(24);
    surfaceFormat.setStencilBufferSize(8);
    setFormat(surfaceFormat);

    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
}

CadViewer::~CadViewer()
{
    if (context() != nullptr)
    {
        makeCurrent();
        clearAllBuffers();
        m_gridVao.destroy();
        m_gridVbo.destroy();
        m_axisVao.destroy();
        m_axisVbo.destroy();
        doneCurrent();
    }
}

void CadViewer::setDocument(CadDocument* document)
{
    m_scene = document;
    m_selectedEntityId = 0;
    m_buffersDirty = true;
    updateSceneBounds();

    if (m_glInitialized)
    {
        makeCurrent();
        rebuildAllBuffers();
        doneCurrent();
    }

    fitScene();
    update();
}

void CadViewer::fitScene()
{
    updateSceneBounds();

    if (!m_hasSceneBounds)
    {
        return;
    }

    m_camera.fitAll(m_sceneMin, m_sceneMax, aspectRatio());
    update();
}

void CadViewer::zoomIn(float factor)
{
    m_camera.zoom(factor);
    update();
}

void CadViewer::zoomOut(float factor)
{
    if (factor <= 0.0f)
    {
        return;
    }

    m_camera.zoom(1.0f / factor);
    update();
}

QVector3D CadViewer::screenToWorld(const QPoint& screenPos, float depth) const
{
    const float x = (2.0f * static_cast<float>(screenPos.x()) / std::max(1, m_viewportWidth)) - 1.0f;
    const float y = 1.0f - (2.0f * static_cast<float>(screenPos.y()) / std::max(1, m_viewportHeight));

    const QMatrix4x4 inverse = m_camera.viewProjectionMatrix(aspectRatio()).inverted();
    const QVector4D world = inverse * QVector4D(x, y, depth, 1.0f);

    if (qFuzzyIsNull(world.w()))
    {
        return QVector3D();
    }

    return world.toVector3DAffine();
}

QPoint CadViewer::worldToScreen(const QVector3D& worldPos) const
{
    const QPointF screenPoint = projectToScreen
    (
        worldPos,
        m_camera.viewProjectionMatrix(aspectRatio()),
        m_viewportWidth,
        m_viewportHeight
    );

    return screenPoint.toPoint();
}

void CadViewer::initializeGL()
{
    initializeOpenGLFunctions();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glClearColor(0.08f, 0.09f, 0.11f, 1.0f);

    initShaders();
    initGridBuffer();
    initAxisBuffer();
    initSelectionBoxBuffer();

    m_glInitialized = true;
    rebuildAllBuffers();
    fitScene();
}

void CadViewer::resizeGL(int w, int h)
{
    m_viewportWidth = std::max(1, w);
    m_viewportHeight = std::max(1, h);
}

void CadViewer::paintGL()
{
    glViewport(0, 0, m_viewportWidth, m_viewportHeight);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (!m_glInitialized)
    {
        return;
    }

    if (m_buffersDirty)
    {
        rebuildAllBuffers();
    }

    renderGrid();
    renderAxis();
    renderEntities();
}

void CadViewer::mousePressEvent(QMouseEvent* event)
{
    m_lastMousePos = event->pos();

    if (event->button() == Qt::MiddleButton)
    {
        m_interactionMode = (event->modifiers() & Qt::ShiftModifier) != 0
            ? ViewInteractionMode::Orbiting
            : ViewInteractionMode::Panning;
        return;
    }

    if (event->button() == Qt::LeftButton)
    {
        m_selectedEntityId = pickEntity(event->pos());
        update();
        return;
    }

    QOpenGLWidget::mousePressEvent(event);
}

void CadViewer::mouseMoveEvent(QMouseEvent* event)
{
    const QPoint delta = event->pos() - m_lastMousePos;
    m_lastMousePos = event->pos();

    switch (m_interactionMode)
    {
    case ViewInteractionMode::Panning:
        m_camera.pan(-delta.x() * pixelToWorldScale(), delta.y() * pixelToWorldScale());
        update();
        return;
    case ViewInteractionMode::Orbiting:
        m_camera.orbit(-delta.x() * 0.4f, -delta.y() * 0.4f);
        update();
        return;
    default:
        break;
    }

    QOpenGLWidget::mouseMoveEvent(event);
}

void CadViewer::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::MiddleButton)
    {
        m_interactionMode = ViewInteractionMode::Idle;
        return;
    }

    QOpenGLWidget::mouseReleaseEvent(event);
}

void CadViewer::wheelEvent(QWheelEvent* event)
{
    const float factor = event->angleDelta().y() > 0 ? 1.1f : (1.0f / 1.1f);
    const QVector3D nearPoint = screenToWorld(event->position().toPoint(), -1.0f);
    const QVector3D farPoint = screenToWorld(event->position().toPoint(), 1.0f);
    const QVector3D rayDirection = farPoint - nearPoint;
    const QVector3D planeNormal = m_camera.forwardDirection();

    QVector3D anchor = m_camera.target;
    const float denominator = QVector3D::dotProduct(rayDirection, planeNormal);

    if (!qFuzzyIsNull(denominator))
    {
        const float t = QVector3D::dotProduct(m_camera.target - nearPoint, planeNormal) / denominator;
        anchor = nearPoint + rayDirection * t;
    }

    m_camera.zoomAtPoint(factor, anchor, aspectRatio());
    update();
    event->accept();
}

void CadViewer::keyPressEvent(QKeyEvent* event)
{
    switch (event->key())
    {
    case Qt::Key_F:
        fitScene();
        return;
    case Qt::Key_Plus:
    case Qt::Key_Equal:
        zoomIn();
        return;
    case Qt::Key_Minus:
    case Qt::Key_Underscore:
        zoomOut();
        return;
    default:
        break;
    }

    QOpenGLWidget::keyPressEvent(event);
}

void CadViewer::initShaders()
{
    static constexpr const char* vertexShaderSource = R"(
        #version 450 core
        layout(location = 0) in vec3 aPosition;
        uniform mat4 uMvp;
        uniform float uPointSize;
        void main()
        {
            gl_Position = uMvp * vec4(aPosition, 1.0);
            gl_PointSize = uPointSize;
        }
    )";

    static constexpr const char* fragmentShaderSource = R"(
        #version 450 core
        uniform vec3 uColor;
        out vec4 fragColor;
        void main()
        {
            fragColor = vec4(uColor, 1.0);
        }
    )";

    m_entityShader = std::make_unique<QOpenGLShaderProgram>();
    m_entityShader->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    m_entityShader->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    m_entityShader->link();

    m_gridShader = std::make_unique<QOpenGLShaderProgram>();
    m_gridShader->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    m_gridShader->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    m_gridShader->link();
}

void CadViewer::initGridBuffer()
{
    std::vector<QVector3D> vertices;
    constexpr int gridHalfCount = 20;
    constexpr float gridStep = 100.0f;
    constexpr float gridExtent = gridHalfCount * gridStep;

    vertices.reserve((gridHalfCount * 2 + 1) * 4);

    for (int i = -gridHalfCount; i <= gridHalfCount; ++i)
    {
        const float offset = i * gridStep;
        vertices.emplace_back(offset, -gridExtent, 0.0f);
        vertices.emplace_back(offset, gridExtent, 0.0f);
        vertices.emplace_back(-gridExtent, offset, 0.0f);
        vertices.emplace_back(gridExtent, offset, 0.0f);
    }

    m_gridVertexCount = static_cast<int>(vertices.size());

    m_gridVao.create();
    m_gridVao.bind();

    m_gridVbo.create();
    m_gridVbo.bind();
    m_gridVbo.allocate(vertices.data(), m_gridVertexCount * static_cast<int>(sizeof(QVector3D)));

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);

    m_gridVbo.release();
    m_gridVao.release();
}

void CadViewer::initAxisBuffer()
{
    constexpr float axisLength = 300.0f;
    const QVector3D vertices[]
    {
        QVector3D(0.0f, 0.0f, 0.0f), QVector3D(axisLength, 0.0f, 0.0f),
        QVector3D(0.0f, 0.0f, 0.0f), QVector3D(0.0f, axisLength, 0.0f),
        QVector3D(0.0f, 0.0f, 0.0f), QVector3D(0.0f, 0.0f, axisLength)
    };

    m_axisVertexCount = 6;

    m_axisVao.create();
    m_axisVao.bind();

    m_axisVbo.create();
    m_axisVbo.bind();
    m_axisVbo.allocate(vertices, static_cast<int>(sizeof(vertices)));

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);

    m_axisVbo.release();
    m_axisVao.release();
}

void CadViewer::initSelectionBoxBuffer()
{
}

void CadViewer::uploadEntity(const CadItem* entity)
{
    if (entity == nullptr || entity->m_geometry.vertices.isEmpty())
    {
        return;
    }

    const EntityId id = toEntityId(entity);
    removeEntityBuffer(id);

    EntityGpuBuffer& gpuBuffer = m_entityBuffers[id];
    gpuBuffer.vertexCount = entity->m_geometry.vertices.size();
    gpuBuffer.primitiveType = primitiveTypeForEntity(entity);
    gpuBuffer.color = QVector3D(entity->m_color.redF(), entity->m_color.greenF(), entity->m_color.blueF());

    gpuBuffer.vao.create();
    gpuBuffer.vao.bind();

    gpuBuffer.vbo.create();
    gpuBuffer.vbo.bind();
    gpuBuffer.vbo.allocate
    (
        entity->m_geometry.vertices.constData(),
        gpuBuffer.vertexCount * static_cast<int>(sizeof(QVector3D))
    );

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), nullptr);

    gpuBuffer.vbo.release();
    gpuBuffer.vao.release();
}

void CadViewer::removeEntityBuffer(EntityId id)
{
    const auto it = m_entityBuffers.find(id);

    if (it == m_entityBuffers.end())
    {
        return;
    }

    it->second.vao.destroy();
    it->second.vbo.destroy();
    m_entityBuffers.erase(it);
}

void CadViewer::rebuildAllBuffers()
{
    clearAllBuffers();
    updateSceneBounds();

    if (!m_glInitialized || m_scene == nullptr)
    {
        m_buffersDirty = false;
        return;
    }

    for (CadItem* entity : m_scene->m_entities)
    {
        uploadEntity(entity);
    }

    m_buffersDirty = false;
}

void CadViewer::clearAllBuffers()
{
    for (auto& [id, buffer] : m_entityBuffers)
    {
        Q_UNUSED(id);
        buffer.vao.destroy();
        buffer.vbo.destroy();
    }

    m_entityBuffers.clear();
}

void CadViewer::renderGrid()
{
    if (m_gridVertexCount <= 0 || !m_gridShader)
    {
        return;
    }

    m_gridShader->bind();
    m_gridShader->setUniformValue("uMvp", m_camera.viewProjectionMatrix(aspectRatio()));
    m_gridShader->setUniformValue("uColor", QVector3D(0.22f, 0.24f, 0.28f));
    m_gridShader->setUniformValue("uPointSize", 1.0f);

    m_gridVao.bind();
    glDrawArrays(GL_LINES, 0, m_gridVertexCount);
    m_gridVao.release();

    m_gridShader->release();
}

void CadViewer::renderAxis()
{
    if (m_axisVertexCount <= 0 || !m_gridShader)
    {
        return;
    }

    m_gridShader->bind();
    m_gridShader->setUniformValue("uMvp", m_camera.viewProjectionMatrix(aspectRatio()));
    m_gridShader->setUniformValue("uPointSize", 1.0f);

    m_axisVao.bind();

    m_gridShader->setUniformValue("uColor", QVector3D(0.95f, 0.30f, 0.25f));
    glDrawArrays(GL_LINES, 0, 2);

    m_gridShader->setUniformValue("uColor", QVector3D(0.25f, 0.85f, 0.35f));
    glDrawArrays(GL_LINES, 2, 2);

    m_gridShader->setUniformValue("uColor", QVector3D(0.30f, 0.55f, 0.95f));
    glDrawArrays(GL_LINES, 4, 2);

    m_axisVao.release();
    m_gridShader->release();
}

void CadViewer::renderEntities()
{
    if (m_scene == nullptr || !m_entityShader)
    {
        return;
    }

    m_entityShader->bind();
    m_entityShader->setUniformValue("uMvp", m_camera.viewProjectionMatrix(aspectRatio()));

    for (CadItem* entity : m_scene->m_entities)
    {
        const EntityId id = toEntityId(entity);
        const auto it = m_entityBuffers.find(id);

        if (it == m_entityBuffers.end())
        {
            continue;
        }

        EntityGpuBuffer& buffer = it->second;
        const bool isSelected = id == m_selectedEntityId;
        const QVector3D color = isSelected ? QVector3D(1.0f, 0.80f, 0.15f) : buffer.color;
        const float pointSize = buffer.primitiveType == GL_POINTS ? (isSelected ? 12.0f : 8.0f) : 1.0f;

        m_entityShader->setUniformValue("uColor", color);
        m_entityShader->setUniformValue("uPointSize", pointSize);

        buffer.vao.bind();
        glDrawArrays(buffer.primitiveType, 0, buffer.vertexCount);
        buffer.vao.release();
    }

    m_entityShader->release();
}

void CadViewer::updateSceneBounds()
{
    m_hasSceneBounds = false;

    if (m_scene == nullptr || m_scene->m_entities.isEmpty())
    {
        return;
    }

    QVector3D minPoint
    (
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max()
    );

    QVector3D maxPoint
    (
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max()
    );

    for (CadItem* entity : m_scene->m_entities)
    {
        for (const QVector3D& vertex : entity->m_geometry.vertices)
        {
            minPoint.setX(std::min(minPoint.x(), vertex.x()));
            minPoint.setY(std::min(minPoint.y(), vertex.y()));
            minPoint.setZ(std::min(minPoint.z(), vertex.z()));

            maxPoint.setX(std::max(maxPoint.x(), vertex.x()));
            maxPoint.setY(std::max(maxPoint.y(), vertex.y()));
            maxPoint.setZ(std::max(maxPoint.z(), vertex.z()));
        }
    }

    if (minPoint.x() > maxPoint.x())
    {
        return;
    }

    m_sceneMin = minPoint;
    m_sceneMax = maxPoint;
    m_hasSceneBounds = true;
}

EntityId CadViewer::pickEntity(const QPoint& screenPos) const
{
    if (m_scene == nullptr)
    {
        return 0;
    }

    const QMatrix4x4 viewProjection = m_camera.viewProjectionMatrix(aspectRatio());
    const QPointF clickPoint(screenPos);
    const float maxDistanceSquared = kPickThresholdPixels * kPickThresholdPixels;

    EntityId bestId = 0;
    float bestDistanceSquared = maxDistanceSquared;

    for (CadItem* entity : m_scene->m_entities)
    {
        const auto& vertices = entity->m_geometry.vertices;

        if (vertices.isEmpty())
        {
            continue;
        }

        float entityDistanceSquared = std::numeric_limits<float>::max();

        if (entity->m_type == DRW::ETYPE::POINT || vertices.size() == 1)
        {
            const QPointF point = projectToScreen(vertices.front(), viewProjection, m_viewportWidth, m_viewportHeight);
            const QPointF delta = clickPoint - point;
            entityDistanceSquared = static_cast<float>(delta.x() * delta.x() + delta.y() * delta.y());
        }
        else
        {
            for (int i = 0; i < vertices.size() - 1; ++i)
            {
                const QPointF start = projectToScreen(vertices.at(i), viewProjection, m_viewportWidth, m_viewportHeight);
                const QPointF end = projectToScreen(vertices.at(i + 1), viewProjection, m_viewportWidth, m_viewportHeight);
                entityDistanceSquared = std::min(entityDistanceSquared, distanceToSegmentSquared(clickPoint, start, end));
            }
        }

        if (entityDistanceSquared <= bestDistanceSquared)
        {
            bestDistanceSquared = entityDistanceSquared;
            bestId = toEntityId(entity);
        }
    }

    return bestId;
}

float CadViewer::aspectRatio() const
{
    return static_cast<float>(m_viewportWidth) / static_cast<float>(std::max(1, m_viewportHeight));
}

float CadViewer::pixelToWorldScale() const
{
    return m_camera.viewHeight / static_cast<float>(std::max(1, m_viewportHeight));
}
