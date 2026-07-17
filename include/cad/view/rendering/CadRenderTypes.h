// 声明 CadRenderTypes 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 渲染数据类型模块，定义顶点、缓存和 transient 图元等核心结构。
#pragma once

#include <cstdint>
#include <functional>
#include <type_traits>

#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QVector>
#include <QVector3D>
#include <QtGlobal>

#include "core/geometry/GeometryTypes.h"

class CadItem;
struct RenderEntityKey;

namespace CadViewerUtils
{
    RenderEntityKey toRenderEntityKey(const CadItem* entity);
}

struct RenderEntityKey final
{
    constexpr RenderEntityKey() noexcept = default;

    constexpr bool valid() const noexcept
    {
        return m_value != 0;
    }

    friend constexpr bool operator==(RenderEntityKey left, RenderEntityKey right) noexcept
    {
        return left.m_value == right.m_value;
    }

    friend constexpr bool operator!=(RenderEntityKey left, RenderEntityKey right) noexcept
    {
        return !(left == right);
    }

private:
    explicit constexpr RenderEntityKey(std::uintptr_t value) noexcept
        : m_value(value)
    {
    }

    std::uintptr_t m_value = 0;

    friend RenderEntityKey CadViewerUtils::toRenderEntityKey(const CadItem* entity);
    friend struct RenderEntityKeyHash;
    friend size_t qHash(RenderEntityKey key, size_t seed) noexcept;
};

struct RenderEntityKeyHash final
{
    std::size_t operator()(RenderEntityKey key) const noexcept
    {
        return std::hash<std::uintptr_t>{}(key.m_value);
    }
};

inline size_t qHash(RenderEntityKey key, size_t seed = 0) noexcept
{
    return ::qHash(static_cast<quintptr>(key.m_value), seed);
}

static_assert(!std::is_convertible_v<RenderEntityKey, cadcam::geometry::EntityId>);
static_assert(!std::is_convertible_v<cadcam::geometry::EntityId, RenderEntityKey>);

// 单个实体对应的一组 GPU 资源。
struct EntityGpuBuffer
{
    // 顶点缓冲。
    QOpenGLBuffer vbo{ QOpenGLBuffer::VertexBuffer };

    // 顶点数组对象，封装顶点属性绑定状态。
    QOpenGLVertexArrayObject vao;

    // 顶点数量。
    int vertexCount = 0;

    // OpenGL 图元类型，如 GL_LINES / GL_LINE_STRIP / GL_POINTS。
    GLenum primitiveType = GL_LINE_STRIP;

    // 实体绘制颜色（RGB，0~1）。
    QVector3D color = { 1.0f, 1.0f, 1.0f };
};

struct TransientPrimitive
{
    QVector<QVector3D> vertices;
    GLenum primitiveType = GL_LINE_STRIP;
    QVector3D color = { 0.25f, 0.85f, 1.0f };
    float opacity = 1.0f;
    float pointSize = 1.0f;
    bool roundPoint = false;
};
