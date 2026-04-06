#include "pch.h"

#include "CadOpenGLState.h"

#include <QOpenGLContext>
#include <QOpenGLFunctions>

namespace
{
    QOpenGLFunctions* currentFunctions()
    {
        if (QOpenGLContext* context = QOpenGLContext::currentContext())
        {
            return context->functions();
        }

        return nullptr;
    }
}

void CadOpenGLState::initializeDefaults()
{
    QOpenGLFunctions* functions = currentFunctions();

    if (functions == nullptr)
    {
        return;
    }

    functions->glEnable(GL_DEPTH_TEST);
    functions->glEnable(GL_BLEND);
    functions->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    functions->glEnable(GL_PROGRAM_POINT_SIZE);
    functions->glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
}

void CadOpenGLState::prepareFrame(int framebufferWidth, int framebufferHeight)
{
    QOpenGLFunctions* functions = currentFunctions();

    if (functions == nullptr)
    {
        return;
    }

    functions->glViewport(0, 0, framebufferWidth, framebufferHeight);
    functions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}
