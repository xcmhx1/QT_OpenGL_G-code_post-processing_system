#pragma once

#include <memory>
#include <vector>

#include <QMatrix4x4>
#include <QOpenGLShaderProgram>

#include "CadRenderTypes.h"

class CadItem;
class CadSceneRenderCache;

class CadEntityRenderer
{
public:
    static void renderEntities
    (
        QOpenGLShaderProgram& shader,
        const QMatrix4x4& mvp,
        const std::vector<std::unique_ptr<CadItem>>& entities,
        CadSceneRenderCache& sceneRenderCache,
        EntityId selectedEntityId
    );
};
