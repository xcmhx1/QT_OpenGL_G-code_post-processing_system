#include "pch.h"

#include "CadToolPanelWidget.h"

#include <QComboBox>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QSizePolicy>
#include <QToolButton>
#include <QVBoxLayout>

namespace
{
    constexpr int kColorByLayer = 256;
    constexpr int kColorTrueColor = -1;
    constexpr int kPanelHeight = 108;
    constexpr int kComboHeight = 24;
    constexpr int kButtonHeight = 24;

    void addColorOption(QComboBox* comboBox, const QString& text, int colorIndex)
    {
        comboBox->addItem(text, colorIndex);
    }
}

CadToolPanelWidget::CadToolPanelWidget(QWidget* parent)
    : QWidget(parent)
{
    buildUi();
}

void CadToolPanelWidget::setLayerNames(const QStringList& layerNames)
{
    const QString currentLayerName = m_layerComboBox->currentText().trimmed().isEmpty()
        ? QStringLiteral("0")
        : m_layerComboBox->currentText().trimmed();

    const QString currentPropertyLayerName = m_propertyLayerComboBox->currentText().trimmed().isEmpty()
        ? QStringLiteral("0")
        : m_propertyLayerComboBox->currentText().trimmed();

    m_updatingUi = true;
    m_layerComboBox->clear();
    m_propertyLayerComboBox->clear();
    m_layerComboBox->addItems(layerNames);
    m_propertyLayerComboBox->addItems(layerNames);
    m_updatingUi = false;

    setActiveLayerName(!currentPropertyLayerName.isEmpty() ? currentPropertyLayerName : currentLayerName);
}

void CadToolPanelWidget::setLayerStatusText(const QString& text)
{
    m_layerStatusLabel->setText(text);
}

void CadToolPanelWidget::setPropertyStatusText(const QString& text)
{
    m_propertyStatusLabel->setText(text);
}

void CadToolPanelWidget::setActiveLayerName(const QString& layerName)
{
    const QString normalizedLayerName = layerName.trimmed().isEmpty() ? QStringLiteral("0") : layerName.trimmed();
    m_updatingUi = true;
    m_layerComboBox->setEditText(normalizedLayerName);
    m_propertyLayerComboBox->setEditText(normalizedLayerName);
    m_updatingUi = false;
}

void CadToolPanelWidget::setActiveColorState(const QColor& color, int colorIndex)
{
    m_updatingUi = true;
    setComboCurrentByData(m_colorComboBox, colorIndex);
    m_updatingUi = false;
    updateColorSwatch(color);
}

void CadToolPanelWidget::setMoveEnabled(bool enabled)
{
    m_moveButton->setEnabled(enabled);
}

void CadToolPanelWidget::buildUi()
{
    QHBoxLayout* rootLayout = new QHBoxLayout(this);
    rootLayout->setContentsMargins(0, 0, 0, 0);
    rootLayout->setSpacing(6);

    rootLayout->addWidget(buildPanelFrame(QStringLiteral("绘图"), buildDrawPanel(), 344), 0, Qt::AlignLeft | Qt::AlignTop);
    rootLayout->addWidget(buildPanelFrame(QStringLiteral("修改"), buildModifyPanel(), 96), 0, Qt::AlignLeft | Qt::AlignTop);
    rootLayout->addWidget(buildPanelFrame(QStringLiteral("图层"), buildLayerPanel(), 176), 0, Qt::AlignLeft | Qt::AlignTop);
    rootLayout->addWidget(buildPanelFrame(QStringLiteral("特性"), buildPropertyPanel(), 226), 0, Qt::AlignLeft | Qt::AlignTop);
    rootLayout->addStretch(1);
}

QGroupBox* CadToolPanelWidget::buildPanelFrame(const QString& title, QWidget* contentWidget, int preferredWidth)
{
    QGroupBox* panel = new QGroupBox(title, this);
    panel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    panel->setMinimumWidth(preferredWidth);
    panel->setMaximumWidth(preferredWidth);
    panel->setMinimumHeight(kPanelHeight);
    panel->setMaximumHeight(kPanelHeight);

    QVBoxLayout* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(6, 8, 6, 6);
    layout->addWidget(contentWidget);
    return panel;
}

QWidget* CadToolPanelWidget::buildDrawPanel()
{
    QWidget* panel = new QWidget(this);
    QGridLayout* layout = new QGridLayout(panel);
    layout->setContentsMargins(2, 2, 2, 2);
    layout->setHorizontalSpacing(4);
    layout->setVerticalSpacing(4);

    addDrawButton(panel, QStringLiteral("点"), DrawType::Point, 0, 0);
    addDrawButton(panel, QStringLiteral("直线"), DrawType::Line, 0, 1);
    addDrawButton(panel, QStringLiteral("圆"), DrawType::Circle, 0, 2);
    addDrawButton(panel, QStringLiteral("圆弧"), DrawType::Arc, 0, 3);
    addDrawButton(panel, QStringLiteral("椭圆"), DrawType::Ellipse, 1, 0);
    addDrawButton(panel, QStringLiteral("多段线"), DrawType::Polyline, 1, 1);
    addDrawButton(panel, QStringLiteral("轻量多段线"), DrawType::LWPolyline, 1, 2);
    layout->setColumnStretch(4, 1);
    return panel;
}

QWidget* CadToolPanelWidget::buildModifyPanel()
{
    QWidget* panel = new QWidget(this);
    QHBoxLayout* layout = new QHBoxLayout(panel);
    layout->setContentsMargins(2, 2, 2, 2);

    m_moveButton = new QToolButton(panel);
    m_moveButton->setText(QStringLiteral("移动"));
    m_moveButton->setToolButtonStyle(Qt::ToolButtonTextOnly);
    m_moveButton->setFixedSize(68, kButtonHeight);
    m_moveButton->setEnabled(false);
    connect(m_moveButton, &QToolButton::clicked, this, &CadToolPanelWidget::moveRequested);

    layout->addWidget(m_moveButton);
    layout->addStretch(1);
    return panel;
}

QWidget* CadToolPanelWidget::buildLayerPanel()
{
    QWidget* panel = new QWidget(this);
    QVBoxLayout* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(2, 2, 2, 2);
    layout->setSpacing(4);

    m_layerStatusLabel = new QLabel(QStringLiteral("当前默认绘图图层"), panel);
    m_layerStatusLabel->setWordWrap(false);
    m_layerStatusLabel->setFixedHeight(18);
    layout->addWidget(m_layerStatusLabel);

    m_layerComboBox = new QComboBox(panel);
    m_layerComboBox->setEditable(true);
    m_layerComboBox->setInsertPolicy(QComboBox::NoInsert);
    m_layerComboBox->setFixedHeight(kComboHeight);
    layout->addWidget(m_layerComboBox);
    layout->addStretch(1);

    connect
    (
        m_layerComboBox,
        QOverload<int>::of(&QComboBox::activated),
        this,
        [this](int)
        {
            commitLayerChange(m_layerComboBox);
        }
    );

    connect
    (
        m_layerComboBox->lineEdit(),
        &QLineEdit::editingFinished,
        this,
        [this]()
        {
            commitLayerChange(m_layerComboBox);
        }
    );

    return panel;
}

QWidget* CadToolPanelWidget::buildPropertyPanel()
{
    QWidget* panel = new QWidget(this);
    QVBoxLayout* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(2, 2, 2, 2);
    layout->setSpacing(4);

    m_propertyStatusLabel = new QLabel(QStringLiteral("当前默认绘图特性"), panel);
    m_propertyStatusLabel->setWordWrap(false);
    m_propertyStatusLabel->setFixedHeight(18);
    layout->addWidget(m_propertyStatusLabel);

    QFormLayout* formLayout = new QFormLayout();
    formLayout->setContentsMargins(0, 0, 0, 0);
    formLayout->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    formLayout->setFormAlignment(Qt::AlignLeft | Qt::AlignTop);
    formLayout->setHorizontalSpacing(4);
    formLayout->setVerticalSpacing(4);

    m_propertyLayerComboBox = new QComboBox(panel);
    m_propertyLayerComboBox->setEditable(true);
    m_propertyLayerComboBox->setInsertPolicy(QComboBox::NoInsert);
    m_propertyLayerComboBox->setFixedHeight(kComboHeight);
    formLayout->addRow(QStringLiteral("图层"), m_propertyLayerComboBox);

    connect
    (
        m_propertyLayerComboBox,
        QOverload<int>::of(&QComboBox::activated),
        this,
        [this](int)
        {
            commitLayerChange(m_propertyLayerComboBox);
        }
    );

    connect
    (
        m_propertyLayerComboBox->lineEdit(),
        &QLineEdit::editingFinished,
        this,
        [this]()
        {
            commitLayerChange(m_propertyLayerComboBox);
        }
    );

    QWidget* colorRowWidget = new QWidget(panel);
    QHBoxLayout* colorRowLayout = new QHBoxLayout(colorRowWidget);
    colorRowLayout->setContentsMargins(0, 0, 0, 0);
    colorRowLayout->setSpacing(4);

    m_colorComboBox = new QComboBox(colorRowWidget);
    addColorOption(m_colorComboBox, QStringLiteral("ByLayer"), kColorByLayer);
    addColorOption(m_colorComboBox, QStringLiteral("红"), 1);
    addColorOption(m_colorComboBox, QStringLiteral("黄"), 2);
    addColorOption(m_colorComboBox, QStringLiteral("绿"), 3);
    addColorOption(m_colorComboBox, QStringLiteral("青"), 4);
    addColorOption(m_colorComboBox, QStringLiteral("蓝"), 5);
    addColorOption(m_colorComboBox, QStringLiteral("洋红"), 6);
    addColorOption(m_colorComboBox, QStringLiteral("白"), 7);
    addColorOption(m_colorComboBox, QStringLiteral("灰"), 8);
    addColorOption(m_colorComboBox, QStringLiteral("浅灰"), 9);
    addColorOption(m_colorComboBox, QStringLiteral("真彩色"), kColorTrueColor);
    m_colorComboBox->setFixedHeight(kComboHeight);
    colorRowLayout->addWidget(m_colorComboBox, 1);

    m_colorSwatchLabel = new QLabel(colorRowWidget);
    m_colorSwatchLabel->setFixedSize(18, 18);
    m_colorSwatchLabel->setFrameShape(QFrame::Box);
    m_colorSwatchLabel->setFrameShadow(QFrame::Plain);
    colorRowLayout->addWidget(m_colorSwatchLabel, 0, Qt::AlignVCenter);

    formLayout->addRow(QStringLiteral("颜色"), colorRowWidget);
    layout->addLayout(formLayout);
    layout->addStretch(1);

    connect
    (
        m_colorComboBox,
        QOverload<int>::of(&QComboBox::currentIndexChanged),
        this,
        [this](int index)
        {
            if (m_updatingUi || index < 0)
            {
                return;
            }

            emit colorChangeRequested(m_colorComboBox->itemData(index).toInt());
        }
    );

    return panel;
}

void CadToolPanelWidget::addDrawButton(QWidget* parent, const QString& text, DrawType drawType, int row, int column)
{
    QGridLayout* layout = qobject_cast<QGridLayout*>(parent->layout());

    if (layout == nullptr)
    {
        return;
    }

    QToolButton* button = new QToolButton(parent);
    button->setText(text);
    button->setFixedHeight(kButtonHeight);
    button->setMinimumWidth(72);
    button->setToolButtonStyle(Qt::ToolButtonTextOnly);
    connect(button, &QToolButton::clicked, this, [this, drawType]() { emit drawRequested(drawType); });
    layout->addWidget(button, row, column);
}

void CadToolPanelWidget::commitLayerChange(QComboBox* comboBox)
{
    if (m_updatingUi || comboBox == nullptr)
    {
        return;
    }

    const QString layerName = comboBox->currentText().trimmed().isEmpty()
        ? QStringLiteral("0")
        : comboBox->currentText().trimmed();

    emit layerChangeRequested(layerName);
}

void CadToolPanelWidget::updateColorSwatch(const QColor& color)
{
    const QColor swatchColor = color.isValid() ? color : QColor(Qt::white);
    m_colorSwatchLabel->setStyleSheet
    (
        QStringLiteral("background-color: rgb(%1, %2, %3);")
            .arg(swatchColor.red())
            .arg(swatchColor.green())
            .arg(swatchColor.blue())
    );
}

void CadToolPanelWidget::setComboCurrentByData(QComboBox* comboBox, int value)
{
    if (comboBox == nullptr)
    {
        return;
    }

    const int index = comboBox->findData(value);

    if (index >= 0)
    {
        comboBox->setCurrentIndex(index);
        return;
    }

    const int trueColorIndex = comboBox->findData(kColorTrueColor);
    comboBox->setCurrentIndex(trueColorIndex);
}
