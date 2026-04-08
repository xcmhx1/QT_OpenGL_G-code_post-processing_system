#include "pch.h"

#include "Gcode_postprocessing_system.h"
#include "CadBitmapImportDialog.h"
#include "CadBitmapVectorizer.h"
#include "CadItem.h"
#include "GGenerator.h"

#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QStatusBar>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
    constexpr double kSortEpsilon = 1.0e-9;
    constexpr double kTwoPi = 6.28318530717958647692;

    struct ItemTravelInfo
    {
        QVector3D forwardStart;
        QVector3D forwardEnd;
        bool valid = false;
    };

    bool hasSuffix(const QString& filePath, std::initializer_list<const char*> suffixes)
    {
        for (const char* suffix : suffixes)
        {
            if (filePath.endsWith(QString::fromLatin1(suffix), Qt::CaseInsensitive))
            {
                return true;
            }
        }

        return false;
    }

    bool isCadVectorFile(const QString& filePath)
    {
        return hasSuffix(filePath, { ".dxf", ".dwg" });
    }

    bool isBitmapFile(const QString& filePath)
    {
        return hasSuffix(filePath, { ".bmp", ".png", ".jpg", ".jpeg" });
    }

    bool isProcessSortableEntity(const CadItem* item)
    {
        if (item == nullptr)
        {
            return false;
        }

        switch (item->m_type)
        {
        case DRW::ETYPE::LINE:
        case DRW::ETYPE::ARC:
        case DRW::ETYPE::CIRCLE:
        case DRW::ETYPE::ELLIPSE:
        case DRW::ETYPE::POLYLINE:
        case DRW::ETYPE::LWPOLYLINE:
            return true;
        default:
            return false;
        }
    }

    QVector3D ellipsePointAt(const DRW_Ellipse* ellipse, double parameter)
    {
        if (ellipse == nullptr)
        {
            return QVector3D();
        }

        const QVector3D center(ellipse->basePoint.x, ellipse->basePoint.y, ellipse->basePoint.z);
        const QVector3D majorAxis(ellipse->secPoint.x, ellipse->secPoint.y, ellipse->secPoint.z);

        if (majorAxis.lengthSquared() <= kSortEpsilon || ellipse->ratio <= 0.0)
        {
            return QVector3D();
        }

        QVector3D normal(ellipse->extPoint.x, ellipse->extPoint.y, ellipse->extPoint.z);

        if (normal.lengthSquared() <= kSortEpsilon)
        {
            normal = QVector3D(0.0f, 0.0f, 1.0f);
        }
        else
        {
            normal.normalize();
        }

        QVector3D minorAxis = QVector3D::crossProduct(normal, majorAxis);

        if (minorAxis.lengthSquared() <= kSortEpsilon)
        {
            return QVector3D();
        }

        minorAxis.normalize();
        minorAxis *= static_cast<float>(majorAxis.length() * ellipse->ratio);

        return center
            + majorAxis * static_cast<float>(std::cos(parameter))
            + minorAxis * static_cast<float>(std::sin(parameter));
    }

    bool isPointLexicographicallyLess(const QVector3D& left, const QVector3D& right)
    {
        if (left.x() != right.x())
        {
            return left.x() < right.x();
        }

        if (left.y() != right.y())
        {
            return left.y() < right.y();
        }

        return left.z() < right.z();
    }

    ItemTravelInfo buildItemTravelInfo(const CadItem* item)
    {
        ItemTravelInfo info;

        if (!isProcessSortableEntity(item) || item->m_nativeEntity == nullptr)
        {
            return info;
        }

        switch (item->m_type)
        {
        case DRW::ETYPE::LINE:
        {
            const DRW_Line* line = static_cast<const DRW_Line*>(item->m_nativeEntity);
            info.forwardStart = QVector3D(line->basePoint.x, line->basePoint.y, line->basePoint.z);
            info.forwardEnd = QVector3D(line->secPoint.x, line->secPoint.y, line->secPoint.z);
            break;
        }
        case DRW::ETYPE::ARC:
        {
            const DRW_Arc* arc = static_cast<const DRW_Arc*>(item->m_nativeEntity);
            const QVector3D center(arc->basePoint.x, arc->basePoint.y, arc->basePoint.z);
            info.forwardStart = QVector3D
            (
                static_cast<float>(center.x() + std::cos(arc->staangle) * arc->radious),
                static_cast<float>(center.y() + std::sin(arc->staangle) * arc->radious),
                center.z()
            );
            info.forwardEnd = QVector3D
            (
                static_cast<float>(center.x() + std::cos(arc->endangle) * arc->radious),
                static_cast<float>(center.y() + std::sin(arc->endangle) * arc->radious),
                center.z()
            );
            break;
        }
        case DRW::ETYPE::CIRCLE:
        {
            const DRW_Circle* circle = static_cast<const DRW_Circle*>(item->m_nativeEntity);
            info.forwardStart = QVector3D
            (
                static_cast<float>(circle->basePoint.x + circle->radious),
                static_cast<float>(circle->basePoint.y),
                static_cast<float>(circle->basePoint.z)
            );
            info.forwardEnd = info.forwardStart;
            break;
        }
        case DRW::ETYPE::ELLIPSE:
        {
            const DRW_Ellipse* ellipse = static_cast<const DRW_Ellipse*>(item->m_nativeEntity);
            double startParam = ellipse->staparam;
            double endParam = ellipse->endparam;
            const bool isClosed = std::abs(endParam - startParam) < 1.0e-10
                || std::abs(std::abs(endParam - startParam) - kTwoPi) < 1.0e-10;

            if (isClosed)
            {
                endParam = startParam;
            }
            else
            {
                while (endParam <= startParam)
                {
                    endParam += kTwoPi;
                }
            }

            info.forwardStart = ellipsePointAt(ellipse, startParam);
            info.forwardEnd = ellipsePointAt(ellipse, endParam);
            break;
        }
        case DRW::ETYPE::POLYLINE:
        {
            const DRW_Polyline* polyline = static_cast<const DRW_Polyline*>(item->m_nativeEntity);

            if (polyline->vertlist.empty())
            {
                return info;
            }

            const auto& firstVertex = polyline->vertlist.front();
            const auto& lastVertex = polyline->vertlist.back();
            info.forwardStart = QVector3D(firstVertex->basePoint.x, firstVertex->basePoint.y, firstVertex->basePoint.z);
            info.forwardEnd = (polyline->flags & 1) != 0
                ? info.forwardStart
                : QVector3D(lastVertex->basePoint.x, lastVertex->basePoint.y, lastVertex->basePoint.z);
            break;
        }
        case DRW::ETYPE::LWPOLYLINE:
        {
            const DRW_LWPolyline* polyline = static_cast<const DRW_LWPolyline*>(item->m_nativeEntity);

            if (polyline->vertlist.empty())
            {
                return info;
            }

            const auto& firstVertex = polyline->vertlist.front();
            const auto& lastVertex = polyline->vertlist.back();
            const float z = static_cast<float>(polyline->elevation);
            info.forwardStart = QVector3D(static_cast<float>(firstVertex->x), static_cast<float>(firstVertex->y), z);
            info.forwardEnd = (polyline->flags & 1) != 0
                ? info.forwardStart
                : QVector3D(static_cast<float>(lastVertex->x), static_cast<float>(lastVertex->y), z);
            break;
        }
        default:
            return info;
        }

        info.valid = true;
        return info;
    }

    int nextProcessOrder(const CadDocument& document)
    {
        int maxOrder = -1;

        for (const std::unique_ptr<CadItem>& entity : document.m_entities)
        {
            if (entity != nullptr)
            {
                maxOrder = std::max(maxOrder, entity->m_processOrder);
            }
        }

        return maxOrder + 1;
    }
}

Gcode_postprocessing_system::Gcode_postprocessing_system(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::Gcode_postprocessing_systemClass())
{
    ui->setupUi(this);

    m_commandLineWidget = new CadCommandLineWidget(this);
    m_statusPaneWidget = new CadStatusPaneWidget(this);

    if (QVBoxLayout* centralLayout = qobject_cast<QVBoxLayout*>(ui->centralWidget->layout()))
    {
        centralLayout->addWidget(m_commandLineWidget);
        centralLayout->addWidget(m_statusPaneWidget);
    }

    m_editer.setDocument(&m_document);
    ui->openGLWidget->setEditer(&m_editer);
    ui->openGLWidget->setDocument(&m_document);
    ui->openGLWidget->refreshCommandPrompt();

    connect(ui->openGLWidget, &CadViewer::hoveredWorldPositionChanged, m_statusPaneWidget, &CadStatusPaneWidget::setWorldPosition);
    connect(ui->openGLWidget, &CadViewer::commandPromptChanged, m_commandLineWidget, &CadCommandLineWidget::setPrompt);
    connect(ui->openGLWidget, &CadViewer::commandMessageAppended, m_commandLineWidget, &CadCommandLineWidget::appendMessage);
    connect
    (
        ui->openGLWidget,
        &CadViewer::fileDropRequested,
        this,
        [this](const QString& filePath)
        {
            importCadFile(filePath);
        }
    );

    connect
    (
        ui->action_File_Import_Dxf,
        &QAction::triggered,
        this,
        [this]()
        {
            const QString filePath = QFileDialog::getOpenFileName
            (
                this,
                QStringLiteral("导入文件"),
                QString(),
                QStringLiteral("支持文件 (*.dxf *.dwg *.bmp *.png *.jpg *.jpeg);;CAD 文件 (*.dxf *.dwg);;位图文件 (*.bmp *.png *.jpg *.jpeg)")
            );

            if (filePath.isEmpty())
            {
                return;
            }

            importCadFile(filePath);
        }
    );

    connect
    (
        ui->action_File_Import_Image,
        &QAction::triggered,
        this,
        [this]()
        {
            const QString filePath = QFileDialog::getOpenFileName
            (
                this,
                QStringLiteral("导入图片"),
                QString(),
                QStringLiteral("位图文件 (*.bmp *.png *.jpg *.jpeg)")
            );

            if (filePath.isEmpty())
            {
                return;
            }

            importBitmapFile(filePath);
        }
    );

    connect(ui->action_File_Export_G, &QAction::triggered, this, [this]() { exportGCode(); });
    connect(ui->action_Edit_ReversePeocess, &QAction::triggered, this, [this]() { toggleSelectedEntityReverse(); });
    connect(ui->action_Sort_Assign, &QAction::triggered, this, [this]() { assignSelectedEntityProcessOrder(); });
    connect(ui->action_Sort_Smart, &QAction::triggered, this, [this]() { smartSortEntities(); });
}

Gcode_postprocessing_system::~Gcode_postprocessing_system()
{
    delete ui;
}

bool Gcode_postprocessing_system::importCadFile(const QString& filePath)
{
    if (filePath.isEmpty())
    {
        return false;
    }

    if (isCadVectorFile(filePath))
    {
        return importDxfFile(filePath);
    }

    if (isBitmapFile(filePath))
    {
        return importBitmapFile(filePath);
    }

    QMessageBox::warning(this, QStringLiteral("导入失败"), QStringLiteral("当前不支持该文件类型: %1").arg(QFileInfo(filePath).suffix()));
    return false;
}

bool Gcode_postprocessing_system::importDxfFile(const QString& filePath)
{
    m_editer.clearHistory();
    m_document.readDxfDocument(filePath);
    ui->openGLWidget->setDocument(&m_document);
    ui->openGLWidget->appendCommandMessage(QStringLiteral("已导入文件: %1").arg(QFileInfo(filePath).fileName()));
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("已导入: %1").arg(QFileInfo(filePath).fileName()), 5000);

    if (m_document.m_entities.empty())
    {
        QMessageBox::warning(this, QStringLiteral("导入结果"), QStringLiteral("文件已读取，但未生成可显示的 CAD 图元。"));
    }

    return true;
}

bool Gcode_postprocessing_system::importBitmapFile(const QString& filePath)
{
    CadBitmapImportDialog dialog(filePath, this);

    if (!dialog.isReady())
    {
        QMessageBox::warning(this, QStringLiteral("位图导入失败"), dialog.errorMessage());
        return false;
    }

    if (dialog.exec() != QDialog::Accepted)
    {
        return false;
    }

    const CadBitmapImportOptions importOptions = dialog.options();
    CadBitmapImportResult importResult;
    QString errorMessage;

    if (!CadBitmapVectorizer::vectorize(dialog.sourceImage(), importOptions, importResult, &errorMessage))
    {
        QMessageBox::warning(this, QStringLiteral("位图导入失败"), errorMessage);
        return false;
    }

    const bool replaceExisting = importOptions.importMode == CadBitmapImportMode::ReplaceDocument;
    m_editer.clearHistory();

    const int appendedCount = m_document.appendEntities(std::move(importResult.entities), replaceExisting);

    if (importOptions.autoFitScene)
    {
        ui->openGLWidget->fitScene();
    }

    ui->openGLWidget->appendCommandMessage
    (
        QStringLiteral("位图导入完成: %1，图层 %2，%3")
            .arg(QFileInfo(filePath).fileName())
            .arg(importOptions.layerName)
            .arg(importResult.summaryText)
    );
    ui->openGLWidget->refreshCommandPrompt();

    if (appendedCount <= 0)
    {
        QMessageBox::warning(this, QStringLiteral("位图导入结果"), QStringLiteral("位图处理完成，但没有生成可显示的 CAD 图元。"));
        return false;
    }

    statusBar()->showMessage
    (
        QStringLiteral("位图已导入: %1，新增实体 %2").arg(QFileInfo(filePath).fileName()).arg(appendedCount),
        5000
    );

    return true;
}

bool Gcode_postprocessing_system::exportGCode()
{
    if (m_document.m_entities.empty())
    {
        QMessageBox::warning(this, QStringLiteral("导出失败"), QStringLiteral("当前文档为空，无法导出 G 代码。"));
        return false;
    }

    GGenerator generator;
    generator.setDocument(&m_document);

    QString errorMessage;

    if (!generator.generate(this, &errorMessage))
    {
        if (!errorMessage.isEmpty())
        {
            QMessageBox::warning(this, QStringLiteral("导出失败"), errorMessage);
        }

        return false;
    }

    ui->openGLWidget->appendCommandMessage(QStringLiteral("G 代码导出完成。"));
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("G 代码导出完成"), 5000);
    return true;
}

bool Gcode_postprocessing_system::toggleSelectedEntityReverse()
{
    CadItem* selectedItem = ui->openGLWidget->selectedEntity();

    if (selectedItem == nullptr)
    {
        QMessageBox::warning(this, QStringLiteral("反向加工"), QStringLiteral("请先选择一个图元。"));
        return false;
    }

    if (!m_editer.toggleEntityReverse(selectedItem))
    {
        QMessageBox::warning(this, QStringLiteral("反向加工"), QStringLiteral("当前图元的反向加工状态切换失败。"));
        return false;
    }

    const QString reverseStateText = selectedItem->m_isReverse
        ? QStringLiteral("反向")
        : QStringLiteral("正向");

    ui->openGLWidget->appendCommandMessage(QStringLiteral("当前选中图元加工方向已切换为%1。").arg(reverseStateText));
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("加工方向已切换为%1").arg(reverseStateText), 5000);
    return true;
}

bool Gcode_postprocessing_system::assignSelectedEntityProcessOrder()
{
    CadItem* selectedItem = ui->openGLWidget->selectedEntity();

    if (selectedItem == nullptr)
    {
        QMessageBox::warning(this, QStringLiteral("排序"), QStringLiteral("请先选择一个图元。"));
        return false;
    }

    if (!isProcessSortableEntity(selectedItem))
    {
        QMessageBox::warning(this, QStringLiteral("排序"), QStringLiteral("当前图元类型暂不支持加工排序。"));
        return false;
    }

    const int processOrder = nextProcessOrder(m_document);

    if (!m_editer.setEntityProcessOrder(selectedItem, processOrder))
    {
        QMessageBox::warning(this, QStringLiteral("排序"), QStringLiteral("当前图元加工顺序设置失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage(QStringLiteral("当前选中图元已设置为第 %1 个加工对象。").arg(processOrder + 1));
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("已设置加工顺序 #%1").arg(processOrder + 1), 5000);
    return true;
}

bool Gcode_postprocessing_system::smartSortEntities()
{
    if (m_document.m_entities.empty())
    {
        QMessageBox::warning(this, QStringLiteral("智能排序"), QStringLiteral("当前文档为空，无法执行智能排序。"));
        return false;
    }

    std::vector<CadItem*> sortableItems;
    std::vector<ItemTravelInfo> travelInfos;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        const ItemTravelInfo info = buildItemTravelInfo(entity.get());

        if (!info.valid)
        {
            continue;
        }

        sortableItems.push_back(entity.get());
        travelInfos.push_back(info);
    }

    if (sortableItems.empty())
    {
        QMessageBox::warning(this, QStringLiteral("智能排序"), QStringLiteral("当前文档中没有可参与 G 代码排序的图元。"));
        return false;
    }

    std::vector<CadItem*> orderedItems;
    std::vector<int> processOrders;
    std::vector<bool> reverseStates;
    std::vector<bool> visited(sortableItems.size(), false);

    orderedItems.reserve(sortableItems.size());
    processOrders.reserve(sortableItems.size());
    reverseStates.reserve(sortableItems.size());

    int currentIndex = -1;
    bool currentReverse = false;
    QVector3D currentEndPoint;

    for (size_t order = 0; order < sortableItems.size(); ++order)
    {
        int bestIndex = -1;
        bool bestReverse = false;
        double bestDistance = std::numeric_limits<double>::max();
        QVector3D bestStartPoint;
        QVector3D bestEndPoint;

        for (size_t index = 0; index < sortableItems.size(); ++index)
        {
            if (visited[index])
            {
                continue;
            }

            const ItemTravelInfo& info = travelInfos[index];
            const QVector3D forwardStart = info.forwardStart;
            const QVector3D forwardEnd = info.forwardEnd;

            for (bool reverse : { false, true })
            {
                const QVector3D candidateStart = reverse ? forwardEnd : forwardStart;
                const QVector3D candidateEnd = reverse ? forwardStart : forwardEnd;
                const double distance = currentIndex < 0
                    ? 0.0
                    : static_cast<double>((candidateStart - currentEndPoint).lengthSquared());

                const bool shouldReplace = bestIndex < 0
                    || (currentIndex < 0 && isPointLexicographicallyLess(candidateStart, bestStartPoint))
                    || (currentIndex >= 0 && distance < bestDistance - kSortEpsilon)
                    || (currentIndex >= 0 && std::abs(distance - bestDistance) <= kSortEpsilon && isPointLexicographicallyLess(candidateStart, bestStartPoint));

                if (!shouldReplace)
                {
                    continue;
                }

                bestIndex = static_cast<int>(index);
                bestReverse = reverse;
                bestDistance = distance;
                bestStartPoint = candidateStart;
                bestEndPoint = candidateEnd;
            }
        }

        if (bestIndex < 0)
        {
            QMessageBox::warning(this, QStringLiteral("智能排序"), QStringLiteral("智能排序过程中出现无效图元，排序已中止。"));
            return false;
        }

        visited[static_cast<size_t>(bestIndex)] = true;
        orderedItems.push_back(sortableItems[static_cast<size_t>(bestIndex)]);
        processOrders.push_back(static_cast<int>(order));
        reverseStates.push_back(bestReverse);
        currentIndex = bestIndex;
        currentReverse = bestReverse;
        currentEndPoint = bestEndPoint;
        Q_UNUSED(currentReverse);
    }

    if (!m_editer.applyEntityProcessStates(orderedItems, processOrders, reverseStates))
    {
        QMessageBox::warning(this, QStringLiteral("智能排序"), QStringLiteral("智能排序结果写入失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage(QStringLiteral("智能排序完成，共更新 %1 个图元的加工顺序。").arg(orderedItems.size()));
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("智能排序完成，共更新 %1 个图元").arg(orderedItems.size()), 5000);
    return true;
}
