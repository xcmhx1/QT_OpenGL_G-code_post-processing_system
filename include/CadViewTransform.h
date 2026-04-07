// CadViewTransform 头文件
// 声明 CadViewTransform 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 视图变换模块，负责屏幕坐标、世界坐标与相机参数之间的转换。
#pragma once

#include <QPoint>
#include <QVector3D>

#include "CadCamera.h"

namespace CadViewTransform
{
    // 统一计算视口宽高比，内部会对高度为 0 的情况做保护。
    // @param viewportWidth 视口宽度
    // @param viewportHeight 视口高度
    // @return 安全宽高比
    float aspectRatio(int viewportWidth, int viewportHeight);

    // 从屏幕坐标反投影到世界坐标。
    // depth 使用 NDC 深度，-1 对应近平面，1 对应远平面。
    // @param camera 当前轨道相机
    // @param viewportWidth 视口宽度
    // @param viewportHeight 视口高度
    // @param screenPos 屏幕像素坐标
    // @param depth NDC 深度值
    // @return 反投影得到的世界坐标
    QVector3D screenToWorld
    (
        const OrbitalCamera& camera,
        int viewportWidth,
        int viewportHeight,
        const QPoint& screenPos,
        float depth = 0.0f
    );

    // 把屏幕点投影到 Z=0 地平面，是二维绘图和坐标显示最常用的入口。
    // @param camera 当前轨道相机
    // @param viewportWidth 视口宽度
    // @param viewportHeight 视口高度
    // @param screenPos 屏幕像素坐标
    // @return 地平面交点世界坐标
    QVector3D screenToGroundPlane
    (
        const OrbitalCamera& camera,
        int viewportWidth,
        int viewportHeight,
        const QPoint& screenPos
    );

    // 把屏幕点投影到任意固定 Z 平面。
    // @param camera 当前轨道相机
    // @param viewportWidth 视口宽度
    // @param viewportHeight 视口高度
    // @param screenPos 屏幕像素坐标
    // @param planeZ 目标平面的 Z 高度
    // @return 指定平面交点世界坐标
    QVector3D screenToPlane
    (
        const OrbitalCamera& camera,
        int viewportWidth,
        int viewportHeight,
        const QPoint& screenPos,
        float planeZ
    );

    // 把世界坐标投影回屏幕像素坐标。
    // @param camera 当前轨道相机
    // @param viewportWidth 视口宽度
    // @param viewportHeight 视口高度
    // @param worldPos 世界坐标
    // @return 对应屏幕像素坐标
    QPoint worldToScreen
    (
        const OrbitalCamera& camera,
        int viewportWidth,
        int viewportHeight,
        const QVector3D& worldPos
    );

    // 估算一个屏幕像素在世界空间中对应的长度，当前主要用于平移和拾取辅助。
    // @param camera 当前轨道相机
    // @param viewportHeight 视口高度
    // @return 单像素对应的世界长度
    float pixelToWorldScale(const OrbitalCamera& camera, int viewportHeight);
}
