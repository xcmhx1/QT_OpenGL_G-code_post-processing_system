// CadCameraMath 头文件
// 声明 CadCameraMath 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 相机数学模块，提供视图矩阵、投影矩阵和几何计算所需的底层公式。
#pragma once

#include <QQuaternion>
#include <QVector3D>

namespace CadCameraMath
{
    // 获取世界坐标系向上方向
    QVector3D worldUp();

    // 获取世界坐标系向下方向
    QVector3D worldDown();

    // 获取顶视图约定的上方向
    QVector3D northUp();

    // 获取相机局部前向
    QVector3D localForward();

    // 获取相机局部右向
    QVector3D localRight();

    // 获取相机局部上向
    QVector3D localUp();

    // 获取回退右向量
    QVector3D fallbackRight();

    // 安全归一化向量
    // @param vector 输入向量
    // @param fallback 输入退化时的回退向量
    // @return 归一化结果或回退向量
    QVector3D normalizedOr(const QVector3D& vector, const QVector3D& fallback);

    // 根据前向与上方向构建相机朝向
    // @param forward 目标前向
    // @param preferredUp 首选上方向
    // @param fallbackUp 退化时使用的上方向
    QQuaternion buildOrientationFromForward
    (
        const QVector3D& forward,
        const QVector3D& preferredUp,
        const QVector3D& fallbackUp
    );

    // 归一化四元数，空四元数时返回单位四元数
    // @param quaternion 输入四元数
    // @return 归一化结果
    QQuaternion normalizedQuaternionOrIdentity(const QQuaternion& quaternion);

    // 对齐四元数半球，避免插值和比较中的符号翻转
    // @param previous 上一个朝向
    // @param current 当前候选朝向
    // @return 对齐后的当前朝向
    QQuaternion alignQuaternionHemisphere(const QQuaternion& previous, const QQuaternion& current);

    // 判断朝向是否违反视图极角约束
    // @param orientation 待检测的相机朝向
    // @return 如果违反约束返回 true，否则返回 false
    bool violatesViewConstraint(const QQuaternion& orientation);
}
