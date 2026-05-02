#include "pch.h"

#include "Gcode_postprocessing_system.h"

#include "CadItem.h"
#include "CadViewer.h"

#include <QMessageBox>

#include <algorithm>

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
    if (ui->openGLWidget == nullptr || !ui->openGLWidget->startCopySelected())
    {
        QMessageBox::warning(this, QStringLiteral("复制图元"), QStringLiteral("请先选择图元。"));
        return false;
    }
    statusBar()->showMessage(QStringLiteral("复制参数输入已启动"), 3000);
    return true;
}

bool Gcode_postprocessing_system::rotateSelectedEntity()
{
    if (ui->openGLWidget == nullptr || !ui->openGLWidget->startRotateSelected())
    {
        QMessageBox::warning(this, QStringLiteral("旋转图元"), QStringLiteral("请先选择图元。"));
        return false;
    }
    statusBar()->showMessage(QStringLiteral("旋转参数输入已启动"), 3000);
    return true;
}

bool Gcode_postprocessing_system::scaleSelectedEntity()
{
    if (ui->openGLWidget == nullptr || !ui->openGLWidget->startScaleSelected())
    {
        QMessageBox::warning(this, QStringLiteral("缩放图元"), QStringLiteral("请先选择图元。"));
        return false;
    }
    statusBar()->showMessage(QStringLiteral("缩放参数输入已启动"), 3000);
    return true;
}

bool Gcode_postprocessing_system::arraySelectedEntity()
{
    if (ui->openGLWidget == nullptr || !ui->openGLWidget->startRectangularArraySelected())
    {
        QMessageBox::warning(this, QStringLiteral("阵列图元"), QStringLiteral("请先选择图元。"));
        return false;
    }
    statusBar()->showMessage(QStringLiteral("矩形阵列参数输入已启动"), 3000);
    return true;
}

bool Gcode_postprocessing_system::circularArraySelectedEntity()
{
    if (ui->openGLWidget == nullptr || !ui->openGLWidget->startCircularArraySelected())
    {
        QMessageBox::warning(this, QStringLiteral("环形阵列"), QStringLiteral("请先选择图元。"));
        return false;
    }
    statusBar()->showMessage(QStringLiteral("环形阵列参数输入已启动"), 3000);
    return true;
}

bool Gcode_postprocessing_system::mirrorSelectedEntities()
{
    if (ui->openGLWidget == nullptr || !ui->openGLWidget->startMirrorSelected())
    {
        QMessageBox::warning(this, QStringLiteral("镜像图元"), QStringLiteral("请先选择图元。"));
        return false;
    }
    statusBar()->showMessage(QStringLiteral("镜像参数输入已启动"), 3000);
    return true;
}

bool Gcode_postprocessing_system::offsetSelectedEntity()
{
    if (ui->openGLWidget == nullptr || !ui->openGLWidget->startOffsetSelected())
    {
        QMessageBox::warning(this, QStringLiteral("偏移图元"), QStringLiteral("请先选择图元。"));
        return false;
    }

    statusBar()->showMessage(QStringLiteral("偏移参数输入已启动"), 3000);
    return true;
}

bool Gcode_postprocessing_system::trimSelectedEntity()
{
    if (ui->openGLWidget == nullptr || !ui->openGLWidget->startTrimSelected())
    {
        QMessageBox::warning(this, QStringLiteral("修剪图元"), QStringLiteral("请选中 2 个图元，并保持当前高亮图元作为修剪目标。"));
        return false;
    }
    statusBar()->showMessage(QStringLiteral("修剪参数输入已启动"), 3000);
    return true;
}

bool Gcode_postprocessing_system::extendSelectedEntity()
{
    if (ui->openGLWidget == nullptr || !ui->openGLWidget->startExtendSelected())
    {
        QMessageBox::warning(this, QStringLiteral("延申图元"), QStringLiteral("请选中 2 个图元，并保持当前高亮图元作为延申目标。"));
        return false;
    }
    statusBar()->showMessage(QStringLiteral("延申参数输入已启动"), 3000);
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
    if (ui->openGLWidget == nullptr || !ui->openGLWidget->startFilletSelected())
    {
        QMessageBox::warning(this, QStringLiteral("圆角图元"), QStringLiteral("请选中 2 个图元，并保持当前高亮图元作为第二条边。"));
        return false;
    }
    statusBar()->showMessage(QStringLiteral("圆角参数输入已启动"), 3000);
    return true;
}

bool Gcode_postprocessing_system::chamferSelectedEntities()
{
    if (ui->openGLWidget == nullptr || !ui->openGLWidget->startChamferSelected())
    {
        QMessageBox::warning(this, QStringLiteral("直角（倒角）"), QStringLiteral("请选中 2 个图元，并保持当前高亮图元作为第二条边。"));
        return false;
    }
    statusBar()->showMessage(QStringLiteral("倒角参数输入已启动"), 3000);
    return true;
}
