#include "pch.h"

#include "Gcode_postprocessing_system.h"

#include "CadItem.h"
#include "CadViewer.h"

#include <QInputDialog>
#include <QMessageBox>

#include <algorithm>

namespace
{
    QVector3D geometryBoundsCenter(const QVector<CadItem*>& items)
    {
        if (items.isEmpty())
        {
            return QVector3D();
        }

        QVector3D minPoint;
        QVector3D maxPoint;
        bool initialized = false;

        for (const CadItem* item : items)
        {
            if (item == nullptr || item->m_geometry.vertices.isEmpty())
            {
                continue;
            }

            for (const QVector3D& point : item->m_geometry.vertices)
            {
                if (!initialized)
                {
                    minPoint = point;
                    maxPoint = point;
                    initialized = true;
                    continue;
                }

                minPoint.setX(std::min(minPoint.x(), point.x()));
                minPoint.setY(std::min(minPoint.y(), point.y()));
                minPoint.setZ(std::min(minPoint.z(), point.z()));
                maxPoint.setX(std::max(maxPoint.x(), point.x()));
                maxPoint.setY(std::max(maxPoint.y(), point.y()));
                maxPoint.setZ(std::max(maxPoint.z(), point.z()));
            }
        }

        if (!initialized)
        {
            return QVector3D();
        }

        return QVector3D
        (
            (minPoint.x() + maxPoint.x()) * 0.5f,
            (minPoint.y() + maxPoint.y()) * 0.5f,
            (minPoint.z() + maxPoint.z()) * 0.5f
        );
    }

    bool resolveCurrentAndOtherSelectedItems(CadViewer* viewer, CadItem*& currentItem, CadItem*& otherItem)
    {
        currentItem = nullptr;
        otherItem = nullptr;

        if (viewer == nullptr)
        {
            return false;
        }

        const QVector<CadItem*> selectedItems = viewer->selectedEntities();

        if (selectedItems.size() != 2)
        {
            return false;
        }

        currentItem = viewer->selectedEntity();

        if (currentItem == nullptr || !selectedItems.contains(currentItem))
        {
            currentItem = selectedItems.front();
        }

        otherItem = selectedItems.front() == currentItem ? selectedItems.back() : selectedItems.front();
        return currentItem != nullptr && otherItem != nullptr;
    }
}

bool Gcode_postprocessing_system::toggleSelectedEntityReverse()
{
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

    if (selectedItems.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("反向加工"), QStringLiteral("请先选择图元。"));
        return false;
    }

    int updatedCount = 0;

    for (CadItem* item : selectedItems)
    {
        if (item != nullptr && m_editer.toggleEntityReverse(item))
        {
            ++updatedCount;
        }
    }

    if (updatedCount <= 0)
    {
        QMessageBox::warning(this, QStringLiteral("反向加工"), QStringLiteral("选中图元的反向加工状态切换失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        updatedCount > 1
            ? QStringLiteral("已切换 %1 个图元的加工方向。").arg(updatedCount)
            : QStringLiteral("当前选中图元加工方向已切换。")
    );
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("加工方向切换完成（%1）").arg(updatedCount), 5000);
    return true;
}

bool Gcode_postprocessing_system::deleteSelectedEntity()
{
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

    if (selectedItems.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("删除图元"), QStringLiteral("请先选择图元。"));
        return false;
    }

    int deletedCount = 0;

    for (CadItem* item : selectedItems)
    {
        if (item != nullptr && m_editer.deleteEntity(item))
        {
            ++deletedCount;
        }
    }

    if (deletedCount <= 0)
    {
        QMessageBox::warning(this, QStringLiteral("删除图元"), QStringLiteral("选中图元删除失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        deletedCount > 1
            ? QStringLiteral("已删除 %1 个图元。").arg(deletedCount)
            : QStringLiteral("已删除选中图元。")
    );
    statusBar()->showMessage(QStringLiteral("图元删除完成（%1）").arg(deletedCount), 4000);
    return true;
}

bool Gcode_postprocessing_system::copySelectedEntity()
{
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

    if (selectedItems.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("复制图元"), QStringLiteral("请先选择图元。"));
        return false;
    }

    bool ok = false;
    const double deltaX = QInputDialog::getDouble
    (
        this,
        QStringLiteral("复制图元"),
        QStringLiteral("请输入 X 偏移量:"),
        10.0,
        -1000000.0,
        1000000.0,
        3,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    const double deltaY = QInputDialog::getDouble
    (
        this,
        QStringLiteral("复制图元"),
        QStringLiteral("请输入 Y 偏移量:"),
        10.0,
        -1000000.0,
        1000000.0,
        3,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    int copiedCount = 0;
    const QVector3D delta(static_cast<float>(deltaX), static_cast<float>(deltaY), 0.0f);

    for (CadItem* item : selectedItems)
    {
        if (item != nullptr && m_editer.copyEntity(item, delta))
        {
            ++copiedCount;
        }
    }

    if (copiedCount <= 0)
    {
        QMessageBox::warning(this, QStringLiteral("复制图元"), QStringLiteral("选中图元复制失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("已复制 %1 个图元，偏移量为 (%2, %3)。").arg(copiedCount).arg(deltaX).arg(deltaY)
    );
    statusBar()->showMessage(QStringLiteral("图元复制完成（%1）").arg(copiedCount), 4000);
    return true;
}

bool Gcode_postprocessing_system::rotateSelectedEntity()
{
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

    if (selectedItems.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("旋转图元"), QStringLiteral("请先选择图元。"));
        return false;
    }

    bool ok = false;
    const double angleDegrees = QInputDialog::getDouble
    (
        this,
        QStringLiteral("旋转图元"),
        QStringLiteral("请输入旋转角度（度）:"),
        90.0,
        -3600.0,
        3600.0,
        3,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    const QVector3D basePoint = geometryBoundsCenter(selectedItems);

    int rotatedCount = 0;

    for (CadItem* item : selectedItems)
    {
        if (item != nullptr && m_editer.rotateEntity(item, basePoint, angleDegrees))
        {
            ++rotatedCount;
        }
    }

    if (rotatedCount <= 0)
    {
        QMessageBox::warning(this, QStringLiteral("旋转图元"), QStringLiteral("选中图元旋转失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("已将 %1 个图元绕中心旋转 %2 度。").arg(rotatedCount).arg(angleDegrees)
    );
    statusBar()->showMessage(QStringLiteral("图元旋转完成（%1）").arg(rotatedCount), 4000);
    return true;
}

bool Gcode_postprocessing_system::scaleSelectedEntity()
{
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

    if (selectedItems.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("缩放图元"), QStringLiteral("请先选择图元。"));
        return false;
    }

    bool ok = false;
    const double scaleFactor = QInputDialog::getDouble
    (
        this,
        QStringLiteral("缩放图元"),
        QStringLiteral("请输入缩放倍率:"),
        2.0,
        0.001,
        1000.0,
        3,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    const QVector3D basePoint = geometryBoundsCenter(selectedItems);

    int scaledCount = 0;

    for (CadItem* item : selectedItems)
    {
        if (item != nullptr && m_editer.scaleEntity(item, basePoint, scaleFactor))
        {
            ++scaledCount;
        }
    }

    if (scaledCount <= 0)
    {
        QMessageBox::warning(this, QStringLiteral("缩放图元"), QStringLiteral("选中图元缩放失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("已将 %1 个图元绕中心缩放为 %2 倍。").arg(scaledCount).arg(scaleFactor)
    );
    statusBar()->showMessage(QStringLiteral("图元缩放完成（%1）").arg(scaledCount), 4000);
    return true;
}

bool Gcode_postprocessing_system::arraySelectedEntity()
{
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

    if (selectedItems.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("阵列图元"), QStringLiteral("请先选择图元。"));
        return false;
    }

    bool ok = false;
    const int rowCount = QInputDialog::getInt
    (
        this,
        QStringLiteral("矩形阵列"),
        QStringLiteral("请输入行数:"),
        2,
        1,
        999,
        1,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    const int columnCount = QInputDialog::getInt
    (
        this,
        QStringLiteral("矩形阵列"),
        QStringLiteral("请输入列数:"),
        2,
        1,
        999,
        1,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    if (rowCount == 1 && columnCount == 1)
    {
        QMessageBox::warning(this, QStringLiteral("矩形阵列"), QStringLiteral("行数和列数不能同时为 1。"));
        return false;
    }

    const double rowSpacing = QInputDialog::getDouble
    (
        this,
        QStringLiteral("矩形阵列"),
        QStringLiteral("请输入行间距（Y 方向）:"),
        10.0,
        -1000000.0,
        1000000.0,
        3,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    const double columnSpacing = QInputDialog::getDouble
    (
        this,
        QStringLiteral("矩形阵列"),
        QStringLiteral("请输入列间距（X 方向）:"),
        10.0,
        -1000000.0,
        1000000.0,
        3,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    int arrayedCount = 0;
    const QVector3D rowOffset(0.0f, static_cast<float>(rowSpacing), 0.0f);
    const QVector3D columnOffset(static_cast<float>(columnSpacing), 0.0f, 0.0f);

    for (CadItem* item : selectedItems)
    {
        if (item != nullptr && m_editer.arrayEntity(item, rowCount, columnCount, rowOffset, columnOffset))
        {
            ++arrayedCount;
        }
    }

    if (arrayedCount <= 0)
    {
        QMessageBox::warning(this, QStringLiteral("矩形阵列"), QStringLiteral("选中图元阵列失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("已对 %1 个图元执行 %2 x %3 矩形阵列。").arg(arrayedCount).arg(rowCount).arg(columnCount)
    );
    statusBar()->showMessage(QStringLiteral("矩形阵列完成（%1）").arg(arrayedCount), 4000);
    return true;
}

bool Gcode_postprocessing_system::circularArraySelectedEntity()
{
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

    if (selectedItems.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("环形阵列"), QStringLiteral("请先选择图元。"));
        return false;
    }

    const QVector3D centerPoint = geometryBoundsCenter(selectedItems);
    bool ok = false;
    const int itemCount = QInputDialog::getInt
    (
        this,
        QStringLiteral("环形阵列"),
        QStringLiteral("请输入项目总数:"),
        6,
        2,
        1024,
        1,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    const double totalAngle = QInputDialog::getDouble
    (
        this,
        QStringLiteral("环形阵列"),
        QStringLiteral("请输入填充角度（度）:"),
        360.0,
        -3600.0,
        3600.0,
        3,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    const double centerX = QInputDialog::getDouble
    (
        this,
        QStringLiteral("环形阵列"),
        QStringLiteral("请输入阵列中心 X:"),
        centerPoint.x(),
        -1000000.0,
        1000000.0,
        3,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    const double centerY = QInputDialog::getDouble
    (
        this,
        QStringLiteral("环形阵列"),
        QStringLiteral("请输入阵列中心 Y:"),
        centerPoint.y(),
        -1000000.0,
        1000000.0,
        3,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    const QStringList rotateChoices
    {
        QStringLiteral("旋转副本方向"),
        QStringLiteral("保持原方向")
    };
    const QString rotateChoice = QInputDialog::getItem
    (
        this,
        QStringLiteral("环形阵列"),
        QStringLiteral("请选择阵列方式:"),
        rotateChoices,
        0,
        false,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    const bool rotateItems = rotateChoice == rotateChoices.front();

    if (!m_editer.polarArrayEntities(selectedItems, QVector3D(static_cast<float>(centerX), static_cast<float>(centerY), 0.0f), itemCount, totalAngle, rotateItems))
    {
        QMessageBox::warning(this, QStringLiteral("环形阵列"), QStringLiteral("选中图元环形阵列失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("已对 %1 个图元执行 %2 项环形阵列。").arg(selectedItems.size()).arg(itemCount)
    );
    statusBar()->showMessage(QStringLiteral("环形阵列完成"), 4000);
    return true;
}

bool Gcode_postprocessing_system::mirrorSelectedEntities()
{
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

    if (selectedItems.isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("镜像图元"), QStringLiteral("请先选择图元。"));
        return false;
    }

    const QVector3D centerPoint = geometryBoundsCenter(selectedItems);
    bool ok = false;
    const double firstX = QInputDialog::getDouble
    (
        this,
        QStringLiteral("镜像图元"),
        QStringLiteral("请输入镜像线第一点 X:"),
        centerPoint.x(),
        -1000000.0,
        1000000.0,
        3,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    const double firstY = QInputDialog::getDouble
    (
        this,
        QStringLiteral("镜像图元"),
        QStringLiteral("请输入镜像线第一点 Y:"),
        centerPoint.y() - 10.0,
        -1000000.0,
        1000000.0,
        3,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    const double secondX = QInputDialog::getDouble
    (
        this,
        QStringLiteral("镜像图元"),
        QStringLiteral("请输入镜像线第二点 X:"),
        centerPoint.x(),
        -1000000.0,
        1000000.0,
        3,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    const double secondY = QInputDialog::getDouble
    (
        this,
        QStringLiteral("镜像图元"),
        QStringLiteral("请输入镜像线第二点 Y:"),
        centerPoint.y() + 10.0,
        -1000000.0,
        1000000.0,
        3,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    const bool eraseSource = QMessageBox::question
    (
        this,
        QStringLiteral("镜像图元"),
        QStringLiteral("是否删除原图元？"),
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No
    ) == QMessageBox::Yes;

    if (!m_editer.mirrorEntities
    (
        selectedItems,
        QVector3D(static_cast<float>(firstX), static_cast<float>(firstY), 0.0f),
        QVector3D(static_cast<float>(secondX), static_cast<float>(secondY), 0.0f),
        eraseSource
    ))
    {
        QMessageBox::warning(this, QStringLiteral("镜像图元"), QStringLiteral("选中图元镜像失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage(QStringLiteral("已执行镜像操作。"));
    statusBar()->showMessage(QStringLiteral("镜像完成"), 4000);
    return true;
}

bool Gcode_postprocessing_system::offsetSelectedEntity()
{
    CadItem* targetItem = ui->openGLWidget->selectedEntity();

    if (targetItem == nullptr)
    {
        const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

        if (selectedItems.isEmpty())
        {
            QMessageBox::warning(this, QStringLiteral("偏移图元"), QStringLiteral("请先选择图元。"));
            return false;
        }

        targetItem = selectedItems.front();
    }

    bool ok = false;
    const double distance = QInputDialog::getDouble
    (
        this,
        QStringLiteral("偏移图元"),
        QStringLiteral("请输入偏移距离（正负决定方向）:"),
        10.0,
        -1000000.0,
        1000000.0,
        3,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    if (!m_editer.offsetEntity(targetItem, distance))
    {
        QMessageBox::warning(this, QStringLiteral("偏移图元"), QStringLiteral("当前图元不支持偏移，或偏移失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage(QStringLiteral("已创建偏移图元，距离为 %1。").arg(distance));
    statusBar()->showMessage(QStringLiteral("偏移完成"), 4000);
    return true;
}

bool Gcode_postprocessing_system::trimSelectedEntity()
{
    CadItem* targetItem = nullptr;
    CadItem* boundaryItem = nullptr;

    if (!resolveCurrentAndOtherSelectedItems(ui->openGLWidget, targetItem, boundaryItem))
    {
        QMessageBox::warning(this, QStringLiteral("修剪图元"), QStringLiteral("请选中 2 个图元，并保持当前高亮图元作为修剪目标。"));
        return false;
    }

    bool ok = false;
    const QStringList sideChoices
    {
        QStringLiteral("起点端"),
        QStringLiteral("终点端")
    };
    const QString sideChoice = QInputDialog::getItem
    (
        this,
        QStringLiteral("修剪图元"),
        QStringLiteral("请选择要修剪的端点:"),
        sideChoices,
        1,
        false,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    if (!m_editer.trimEntity(boundaryItem, targetItem, sideChoice == sideChoices.front()))
    {
        QMessageBox::warning(this, QStringLiteral("修剪图元"), QStringLiteral("当前仅支持将直线修剪到直线、圆或圆弧边界。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage(QStringLiteral("已执行修剪操作。"));
    statusBar()->showMessage(QStringLiteral("修剪完成"), 4000);
    return true;
}

bool Gcode_postprocessing_system::extendSelectedEntity()
{
    CadItem* targetItem = nullptr;
    CadItem* boundaryItem = nullptr;

    if (!resolveCurrentAndOtherSelectedItems(ui->openGLWidget, targetItem, boundaryItem))
    {
        QMessageBox::warning(this, QStringLiteral("延申图元"), QStringLiteral("请选中 2 个图元，并保持当前高亮图元作为延申目标。"));
        return false;
    }

    bool ok = false;
    const QStringList sideChoices
    {
        QStringLiteral("起点端"),
        QStringLiteral("终点端")
    };
    const QString sideChoice = QInputDialog::getItem
    (
        this,
        QStringLiteral("延申图元"),
        QStringLiteral("请选择要延申的端点:"),
        sideChoices,
        1,
        false,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    if (!m_editer.extendEntity(boundaryItem, targetItem, sideChoice == sideChoices.front()))
    {
        QMessageBox::warning(this, QStringLiteral("延申图元"), QStringLiteral("当前仅支持将直线延申到直线、圆或圆弧边界。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage(QStringLiteral("已执行延申操作。"));
    statusBar()->showMessage(QStringLiteral("延申完成"), 4000);
    return true;
}

bool Gcode_postprocessing_system::joinSelectedEntities()
{
    const QVector<CadItem*> selectedItems = ui->openGLWidget->selectedEntities();

    if (selectedItems.size() < 2)
    {
        QMessageBox::warning(this, QStringLiteral("合并图元"), QStringLiteral("请至少选择 2 个线性图元。"));
        return false;
    }

    if (!m_editer.joinEntities(selectedItems))
    {
        QMessageBox::warning(this, QStringLiteral("合并图元"), QStringLiteral("当前仅支持合并相接的直线/直线多段线。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage(QStringLiteral("已合并选中图元。"));
    statusBar()->showMessage(QStringLiteral("合并完成"), 4000);
    return true;
}

bool Gcode_postprocessing_system::filletSelectedEntities()
{
    CadItem* secondItem = nullptr;
    CadItem* firstItem = nullptr;

    if (!resolveCurrentAndOtherSelectedItems(ui->openGLWidget, secondItem, firstItem))
    {
        QMessageBox::warning(this, QStringLiteral("圆角图元"), QStringLiteral("请选中 2 个图元，并保持当前高亮图元作为第二条边。"));
        return false;
    }

    bool ok = false;
    const double radius = QInputDialog::getDouble
    (
        this,
        QStringLiteral("圆角图元"),
        QStringLiteral("请输入圆角半径:"),
        5.0,
        0.001,
        1000000.0,
        3,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    if (!m_editer.filletEntities(firstItem, secondItem, radius))
    {
        QMessageBox::warning(this, QStringLiteral("圆角图元"), QStringLiteral("当前仅支持 2 条直线的圆角。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage(QStringLiteral("已执行圆角操作，半径 %1。").arg(radius));
    statusBar()->showMessage(QStringLiteral("圆角完成"), 4000);
    return true;
}

bool Gcode_postprocessing_system::chamferSelectedEntities()
{
    CadItem* secondItem = nullptr;
    CadItem* firstItem = nullptr;

    if (!resolveCurrentAndOtherSelectedItems(ui->openGLWidget, secondItem, firstItem))
    {
        QMessageBox::warning(this, QStringLiteral("直角（倒角）"), QStringLiteral("请选中 2 个图元，并保持当前高亮图元作为第二条边。"));
        return false;
    }

    bool ok = false;
    const double firstDistance = QInputDialog::getDouble
    (
        this,
        QStringLiteral("直角（倒角）"),
        QStringLiteral("请输入第一条边距离:"),
        5.0,
        0.0,
        1000000.0,
        3,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    const double secondDistance = QInputDialog::getDouble
    (
        this,
        QStringLiteral("直角（倒角）"),
        QStringLiteral("请输入第二条边距离:"),
        5.0,
        0.0,
        1000000.0,
        3,
        &ok
    );

    if (!ok)
    {
        return false;
    }

    if (!m_editer.chamferEntities(firstItem, secondItem, firstDistance, secondDistance))
    {
        QMessageBox::warning(this, QStringLiteral("直角（倒角）"), QStringLiteral("当前仅支持 2 条直线的倒角。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("已执行直角（倒角）操作，距离为 %1 / %2。").arg(firstDistance).arg(secondDistance)
    );
    statusBar()->showMessage(QStringLiteral("直角（倒角）完成"), 4000);
    return true;
}
