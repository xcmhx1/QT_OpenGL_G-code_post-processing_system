#include "pch.h"

#include "Gcode_postprocessing_system.h"
#include "CadBitmapImportDialog.h"
#include "CadBitmapVectorizer.h"
#include "CadItem.h"
#include "CadProcessVisualUtils.h"
#include "GGenerator.h"

#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QStatusBar>
#include <QToolBar>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
    constexpr double kSortEpsilon = 1.0e-9;
    constexpr int kColorByLayer = 256;

    QColor colorFromAci(int colorIndex)
    {
        static const QRgb aciStandardColors[] =
        {
            qRgb(0, 0, 0),
            qRgb(255, 0, 0),
            qRgb(255, 255, 0),
            qRgb(0, 255, 0),
            qRgb(0, 255, 255),
            qRgb(0, 0, 255),
            qRgb(255, 0, 255),
            qRgb(255, 255, 255),
            qRgb(128, 128, 128),
            qRgb(192, 192, 192)
        };

        if (colorIndex >= 1 && colorIndex <= 9)
        {
            return QColor(aciStandardColors[colorIndex]);
        }

        if (colorIndex == 0)
        {
            return QColor(Qt::white);
        }

        return QColor();
    }

    QColor colorFromTrueColor(int color24)
    {
        if (color24 < 0)
        {
            return QColor();
        }

        return QColor((color24 >> 16) & 0xFF, (color24 >> 8) & 0xFF, color24 & 0xFF);
    }

    QString entityLayerName(const CadItem* item)
    {
        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            return QStringLiteral("0");
        }

        const QString layerName = QString::fromUtf8(item->m_nativeEntity->layer.c_str()).trimmed();
        return layerName.isEmpty() ? QStringLiteral("0") : layerName;
    }

    int entityColorIndex(const CadItem* item)
    {
        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            return kColorByLayer;
        }

        return item->m_nativeEntity->color24 != -1
            ? -1
            : item->m_nativeEntity->color;
    }

    QColor entityDisplayColor(const CadDocument& document, const CadItem* item)
    {
        if (item == nullptr || item->m_nativeEntity == nullptr)
        {
            return QColor(Qt::white);
        }

        const QColor trueColor = colorFromTrueColor(item->m_nativeEntity->color24);

        if (trueColor.isValid())
        {
            return trueColor;
        }

        if (item->m_nativeEntity->color == kColorByLayer)
        {
            return document.layerColor(entityLayerName(item), item->m_color);
        }

        const QColor aciColor = colorFromAci(item->m_nativeEntity->color);
        return aciColor.isValid() ? aciColor : item->m_color;
    }

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
    connect(ui->action_Sort_Assign, &QAction::triggered, this, [this]() { sortEntitiesByCurrentDirection(); });
    connect(ui->action_Sort_Smart, &QAction::triggered, this, [this]() { smartSortEntities(); });

    initializeToolPanel();
    applyDefaultDrawingProperties();
    syncToolPanelState();
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

    syncToolPanelState();
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

    syncToolPanelState();
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

bool Gcode_postprocessing_system::sortEntitiesByCurrentDirection()
{
    if (m_document.m_entities.empty())
    {
        QMessageBox::warning(this, QStringLiteral("排序"), QStringLiteral("当前文档为空，无法执行排序。"));
        return false;
    }

    std::vector<CadItem*> sortableItems;
    std::vector<CadProcessVisualInfo> visualInfos;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        const CadProcessVisualInfo info = buildProcessVisualInfo(entity.get());

        if (!info.valid)
        {
            continue;
        }

        sortableItems.push_back(entity.get());
        visualInfos.push_back(info);
    }

    if (sortableItems.empty())
    {
        QMessageBox::warning(this, QStringLiteral("排序"), QStringLiteral("当前文档中没有可参与 G 代码排序的图元。"));
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
    QVector3D currentEndPoint;

    for (size_t order = 0; order < sortableItems.size(); ++order)
    {
        int bestIndex = -1;
        double bestDistance = std::numeric_limits<double>::max();
        QVector3D bestStartPoint;
        QVector3D bestEndPoint;

        for (size_t index = 0; index < sortableItems.size(); ++index)
        {
            if (visited[index])
            {
                continue;
            }

            const CadProcessVisualInfo& info = visualInfos[index];
            const QVector3D candidateStart = info.startPoint;
            const QVector3D candidateEnd = info.endPoint;
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
            bestDistance = distance;
            bestStartPoint = candidateStart;
            bestEndPoint = candidateEnd;
        }

        if (bestIndex < 0)
        {
            QMessageBox::warning(this, QStringLiteral("排序"), QStringLiteral("排序过程中出现无效图元，排序已中止。"));
            return false;
        }

        visited[static_cast<size_t>(bestIndex)] = true;
        orderedItems.push_back(sortableItems[static_cast<size_t>(bestIndex)]);
        processOrders.push_back(static_cast<int>(order));
        reverseStates.push_back(sortableItems[static_cast<size_t>(bestIndex)]->m_isReverse);
        currentIndex = bestIndex;
        currentEndPoint = bestEndPoint;
    }

    if (!m_editer.applyEntityProcessStates(orderedItems, processOrders, reverseStates))
    {
        QMessageBox::warning(this, QStringLiteral("排序"), QStringLiteral("排序结果写入失败。"));
        return false;
    }

    ui->openGLWidget->appendCommandMessage(QStringLiteral("排序完成，共更新 %1 个图元的加工顺序，已保留当前加工方向设置。").arg(orderedItems.size()));
    ui->openGLWidget->refreshCommandPrompt();
    statusBar()->showMessage(QStringLiteral("排序完成，共更新 %1 个图元").arg(orderedItems.size()), 5000);
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

    if (!isProcessVisualizable(selectedItem))
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
    std::vector<CadProcessVisualInfo> visualInfos;

    for (const std::unique_ptr<CadItem>& entity : m_document.m_entities)
    {
        if (entity == nullptr)
        {
            continue;
        }

        const CadProcessVisualInfo info = buildProcessVisualInfo(entity.get());

        if (!info.valid)
        {
            continue;
        }

        sortableItems.push_back(entity.get());
        visualInfos.push_back(info);
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

            const CadProcessVisualInfo& info = visualInfos[index];
            const QVector3D forwardStart = info.forwardStartPoint;
            const QVector3D forwardEnd = info.forwardEndPoint;

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

void Gcode_postprocessing_system::initializeToolPanel()
{
    m_toolPanelWidget = new CadToolPanelWidget(this);
    ui->mainToolBar->setMovable(false);
    ui->mainToolBar->setFloatable(false);
    ui->mainToolBar->addWidget(m_toolPanelWidget);

    connect(&m_document, &CadDocument::sceneChanged, this, [this]() { syncToolPanelState(); });
    connect(ui->openGLWidget, &CadViewer::selectedEntityChanged, this, [this](CadItem*) { syncToolPanelState(); });

    connect
    (
        m_toolPanelWidget,
        &CadToolPanelWidget::drawRequested,
        this,
        [this](DrawType drawType)
        {
            applyDefaultDrawingProperties();
            ui->openGLWidget->startDrawing(drawType);
        }
    );

    connect
    (
        m_toolPanelWidget,
        &CadToolPanelWidget::moveRequested,
        this,
        [this]()
        {
            if (!ui->openGLWidget->startMoveSelected())
            {
                statusBar()->showMessage(QStringLiteral("请先选择一个图元再执行移动"), 3000);
            }
        }
    );

    connect
    (
        m_toolPanelWidget,
        &CadToolPanelWidget::layerChangeRequested,
        this,
        [this](const QString& layerName)
        {
            const QString normalizedLayerName = layerName.trimmed().isEmpty() ? QStringLiteral("0") : layerName.trimmed();
            CadItem* selectedItem = ui->openGLWidget->selectedEntity();

            if (selectedItem != nullptr)
            {
                if (m_editer.changeEntityLayer(selectedItem, normalizedLayerName))
                {
                    statusBar()->showMessage(QStringLiteral("图层已更新为 %1").arg(normalizedLayerName), 3000);
                }

                return;
            }

            m_currentLayerName = normalizedLayerName;

            if (m_document.ensureLayerExists(m_currentLayerName))
            {
                m_document.notifySceneChanged();
            }

            applyDefaultDrawingProperties();
            syncToolPanelState();
        }
    );

    connect
    (
        m_toolPanelWidget,
        &CadToolPanelWidget::colorChangeRequested,
        this,
        [this](int colorIndex)
        {
            CadItem* selectedItem = ui->openGLWidget->selectedEntity();

            if (selectedItem != nullptr)
            {
                const QColor targetColor = colorIndex == kColorByLayer
                    ? m_document.layerColor(entityLayerName(selectedItem), entityDisplayColor(m_document, selectedItem))
                    : (colorIndex < 0 ? entityDisplayColor(m_document, selectedItem) : colorFromAci(colorIndex));

                if (m_editer.changeEntityColor(selectedItem, targetColor, colorIndex))
                {
                    statusBar()->showMessage(QStringLiteral("图元颜色已更新"), 3000);
                }

                return;
            }

            m_currentColorIndex = colorIndex;

            if (colorIndex == kColorByLayer)
            {
                m_currentColor = m_document.layerColor(m_currentLayerName, QColor(Qt::white));
            }
            else if (colorIndex >= 0)
            {
                m_currentColor = colorFromAci(colorIndex);
            }

            applyDefaultDrawingProperties();
            syncToolPanelState();
        }
    );
}

void Gcode_postprocessing_system::syncToolPanelState()
{
    if (m_toolPanelWidget == nullptr)
    {
        return;
    }

    const QStringList layerNames = m_document.layerNames();
    m_toolPanelWidget->setLayerNames(layerNames);

    CadItem* selectedItem = ui->openGLWidget->selectedEntity();

    if (selectedItem != nullptr)
    {
        m_toolPanelWidget->setMoveEnabled(true);
        m_toolPanelWidget->setLayerStatusText(QStringLiteral("当前选中图元图层"));
        m_toolPanelWidget->setPropertyStatusText(QStringLiteral("当前选中图元特性"));
        m_toolPanelWidget->setActiveLayerName(entityLayerName(selectedItem));
        m_toolPanelWidget->setActiveColorState(entityDisplayColor(m_document, selectedItem), entityColorIndex(selectedItem));
        return;
    }

    if (m_currentLayerName.trimmed().isEmpty())
    {
        m_currentLayerName = layerNames.isEmpty() ? QStringLiteral("0") : layerNames.front();
    }

    if (m_currentColorIndex == kColorByLayer)
    {
        m_currentColor = m_document.layerColor(m_currentLayerName, QColor(Qt::white));
    }
    else if (m_currentColorIndex >= 0)
    {
        m_currentColor = colorFromAci(m_currentColorIndex);
    }

    m_toolPanelWidget->setMoveEnabled(false);
    m_toolPanelWidget->setLayerStatusText(QStringLiteral("当前默认绘图图层"));
    m_toolPanelWidget->setPropertyStatusText(QStringLiteral("当前默认绘图特性"));
    m_toolPanelWidget->setActiveLayerName(m_currentLayerName);
    m_toolPanelWidget->setActiveColorState(m_currentColor, m_currentColorIndex);
}

void Gcode_postprocessing_system::applyDefaultDrawingProperties()
{
    if (m_currentLayerName.trimmed().isEmpty())
    {
        m_currentLayerName = QStringLiteral("0");
    }

    if (m_currentColorIndex == kColorByLayer)
    {
        m_currentColor = m_document.layerColor(m_currentLayerName, QColor(Qt::white));
    }
    else if (m_currentColorIndex >= 0)
    {
        m_currentColor = colorFromAci(m_currentColorIndex);
    }

    ui->openGLWidget->setDefaultDrawingProperties(m_currentLayerName, m_currentColor, m_currentColorIndex);
}

QString Gcode_postprocessing_system::activeLayerName() const
{
    CadItem* selectedItem = ui->openGLWidget->selectedEntity();
    return selectedItem != nullptr ? entityLayerName(selectedItem) : m_currentLayerName;
}

QColor Gcode_postprocessing_system::activeColor() const
{
    CadItem* selectedItem = ui->openGLWidget->selectedEntity();
    return selectedItem != nullptr ? entityDisplayColor(m_document, selectedItem) : m_currentColor;
}

int Gcode_postprocessing_system::activeColorIndex() const
{
    CadItem* selectedItem = ui->openGLWidget->selectedEntity();
    return selectedItem != nullptr ? entityColorIndex(selectedItem) : m_currentColorIndex;
}
