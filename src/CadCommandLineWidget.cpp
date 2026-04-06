// 实现 CadCommandLineWidget 模块，对应头文件中声明的主要行为和协作流程。
// 命令栏模块，负责命令提示、最新消息和历史记录的界面展示。
#include "pch.h"

#include "CadCommandLineWidget.h"

#include <QApplication>
#include <QBoxLayout>
#include <QEvent>
#include <QFont>
#include <QMouseEvent>
#include <QScrollBar>

CadCommandLineWidget::CadCommandLineWidget(QWidget* parent)
    : QWidget(parent)
{
    setObjectName("CadCommandLineWidget");
    setFocusPolicy(Qt::ClickFocus);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(10, 6, 10, 6);
    layout->setSpacing(4);

    m_summaryLabel = new QLabel(this);
    m_summaryLabel->setWordWrap(false);
    m_summaryLabel->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    m_summaryLabel->setText(QStringLiteral("命令栏就绪"));
    m_summaryLabel->setAttribute(Qt::WA_TransparentForMouseEvents, true);

    QFont summaryFont = m_summaryLabel->font();
    summaryFont.setPointSize(12);
    m_summaryLabel->setFont(summaryFont);

    m_promptLabel = new QLabel(this);
    m_promptLabel->setWordWrap(false);
    m_promptLabel->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
    m_promptLabel->setText(QStringLiteral(" "));

    QFont promptFont = m_promptLabel->font();
    promptFont.setPointSize(12);
    promptFont.setBold(true);
    m_promptLabel->setFont(promptFont);

    m_historyEdit = new QPlainTextEdit(this);
    m_historyEdit->setReadOnly(true);
    m_historyEdit->setUndoRedoEnabled(false);
    m_historyEdit->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_historyEdit->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

    QFont historyFont = m_historyEdit->font();
    historyFont.setPointSize(12);
    m_historyEdit->setFont(historyFont);

    layout->addWidget(m_summaryLabel);
    layout->addWidget(m_promptLabel);
    layout->addWidget(m_historyEdit);

    setStyleSheet
    (
        "#CadCommandLineWidget {"
        "background-color: rgb(33, 36, 40);"
        "border-top: 1px solid rgb(58, 64, 72);"
        "border-bottom: 1px solid rgb(58, 64, 72);"
        "}"
        "#CadCommandLineWidget QLabel {"
        "color: rgb(235, 238, 242);"
        "}"
        "#CadCommandLineWidget QPlainTextEdit {"
        "background-color: rgb(25, 28, 32);"
        "color: rgb(235, 238, 242);"
        "border: 1px solid rgb(66, 72, 80);"
        "padding: 4px;"
        "}"
    );

    refreshSummary();
    refreshHistory();
    setExpanded(false);
    qApp->installEventFilter(this);
}

CadCommandLineWidget::~CadCommandLineWidget()
{
    qApp->removeEventFilter(this);
}

void CadCommandLineWidget::setPrompt(const QString& prompt)
{
    m_prompt = prompt.trimmed();
    refreshSummary();
}

void CadCommandLineWidget::appendMessage(const QString& message)
{
    const QString trimmedMessage = message.trimmed();

    if (trimmedMessage.isEmpty())
    {
        return;
    }

    m_lastMessage = trimmedMessage;
    m_history.append(trimmedMessage);
    refreshHistory();
    refreshSummary();
}

bool CadCommandLineWidget::eventFilter(QObject* watched, QEvent* event)
{
    if (watched == qApp && m_isExpanded && event->type() == QEvent::MouseButtonPress)
    {
        const QMouseEvent* mouseEvent = static_cast<const QMouseEvent*>(event);
        const QPoint globalPos = mouseEvent->globalPosition().toPoint();

        if (!rect().contains(mapFromGlobal(globalPos)))
        {
            setExpanded(false);
        }
    }

    return QWidget::eventFilter(watched, event);
}

void CadCommandLineWidget::mousePressEvent(QMouseEvent* event)
{
    if (!m_isExpanded && event->button() == Qt::LeftButton)
    {
        setExpanded(true);
        m_historyEdit->setFocus();
        event->accept();
        return;
    }

    QWidget::mousePressEvent(event);
}

void CadCommandLineWidget::setExpanded(bool expanded)
{
    m_isExpanded = expanded;
    m_summaryLabel->setVisible(!expanded);
    m_promptLabel->setVisible(expanded);
    m_historyEdit->setVisible(expanded);
    setFixedHeight(expanded ? expandedHeight() : collapsedHeight());

    if (expanded)
    {
        refreshHistory();
    }
}

void CadCommandLineWidget::refreshSummary()
{
    QString summaryText = m_lastMessage;

    if (summaryText.isEmpty())
    {
        summaryText = m_prompt;
    }

    if (summaryText.isEmpty())
    {
        summaryText = QStringLiteral("命令栏就绪");
    }

    m_summaryLabel->setText(summaryText);
    m_promptLabel->setText(m_prompt.isEmpty() ? QStringLiteral("当前无活动命令") : QStringLiteral("当前提示: %1").arg(m_prompt));
}

void CadCommandLineWidget::refreshHistory()
{
    m_historyEdit->setPlainText(m_history.join('\n'));
    QTextCursor cursor = m_historyEdit->textCursor();
    cursor.movePosition(QTextCursor::End);
    m_historyEdit->setTextCursor(cursor);
    m_historyEdit->ensureCursorVisible();
}

int CadCommandLineWidget::collapsedHeight() const
{
    return fontMetrics().height() + 18;
}

int CadCommandLineWidget::expandedHeight() const
{
    const int lineHeight = m_historyEdit->fontMetrics().lineSpacing();
    return lineHeight * 4 + 42;
}
