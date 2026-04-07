// CadCameraUtils 实现文件
// 实现 CadCameraUtils 模块，对应头文件中声明的主要行为和协作流程。
// 相机辅助模块，整理相机相关的公共工具函数和复用计算流程。
#include "pch.h"

#include "CadCameraUtils.h"

#include "CadCameraMath.h"

namespace CadCameraUtils
{
    // 围绕指定枢轴点旋转轨道相机
    // @param camera 待修改的轨道相机
    // @param pivot 旋转中心点
    // @param deltaAzimuth 方位角增量
    // @param deltaElevation 俯仰角增量
    void orbitCameraAroundPivot
    (
        OrbitalCamera& camera,
        const QVector3D& pivot,
        float deltaAzimuth,
        float deltaElevation
    )
    {
        // 保留旋转前的眼点、目标点和朝向，在通过约束检查后再整体提交
        QVector3D eye = camera.eyePosition();
        QVector3D target = camera.target;
        const float distance = camera.distance;
        const QQuaternion previousOrientation = CadCameraMath::normalizedQuaternionOrIdentity(camera.orientation);
        QQuaternion candidateOrientation = previousOrientation;
        QVector3D candidateEye = eye;
        QVector3D candidateTarget = target;

        // yaw 围绕世界 Z 轴转动，同时旋转眼点、目标点和朝向
        if (!qFuzzyIsNull(deltaAzimuth))
        {
            const QQuaternion yawRotation =
                QQuaternion::fromAxisAndAngle(QVector3D(0.0f, 0.0f, 1.0f), deltaAzimuth);

            const QVector3D yawedEye = pivot + yawRotation.rotatedVector(candidateEye - pivot);
            const QVector3D yawedTarget = pivot + yawRotation.rotatedVector(candidateTarget - pivot);
            QQuaternion yawedOrientation = yawRotation * candidateOrientation;
            yawedOrientation.normalize();

            if (!CadCameraMath::violatesViewConstraint(yawedOrientation))
            {
                candidateEye = yawedEye;
                candidateTarget = yawedTarget;
                candidateOrientation = yawedOrientation;
            }
        }

        // pitch 围绕当前相机右轴转动，避免脱离当前观察语义
        if (!qFuzzyIsNull(deltaElevation))
        {
            const QVector3D pitchAxis = CadCameraMath::normalizedOr
            (
                candidateOrientation.rotatedVector(CadCameraMath::localRight()),
                CadCameraMath::localRight()
            );

            const QQuaternion pitchRotation =
                QQuaternion::fromAxisAndAngle(pitchAxis, deltaElevation);

            const QVector3D pitchedEye = pivot + pitchRotation.rotatedVector(candidateEye - pivot);
            const QVector3D pitchedTarget = pivot + pitchRotation.rotatedVector(candidateTarget - pivot);
            QQuaternion pitchedOrientation = pitchRotation * candidateOrientation;
            pitchedOrientation.normalize();

            if (!CadCameraMath::violatesViewConstraint(pitchedOrientation))
            {
                candidateEye = pitchedEye;
                candidateTarget = pitchedTarget;
                candidateOrientation = pitchedOrientation;
            }
        }

        // 通过约束检查后的候选值一次性写回相机
        candidateOrientation.normalize();
        camera.target = candidateTarget;
        camera.distance = distance;
        camera.orientation = CadCameraMath::alignQuaternionHemisphere(previousOrientation, candidateOrientation);
        camera.updateAxesSwappedState();
    }
}
