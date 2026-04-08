// 声明 CadCommandLineWidget 模块，对外暴露当前组件的核心类型、接口和协作边界。
// 命令栏模块，负责命令提示、最新消息和历史记录的界面展示。
#pragma once

#include "AppTheme.h"

#include <QLabel>
#include <QPlainTextEdit>
#include <QPointer>
#include <QStringList>
#include <QWidget>

class CadCommandLineWidget : public QWidget
{
public:
    explicit CadCommandLineWidget(QWidget* parent = nullptr);
    ~CadCommandLineWidget() override;

    void setPrompt(const QString& prompt);
    void appendMessage(const QString& message);
    void setTheme(const AppThemeColors& theme);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;

private:
    void setExpanded(bool expanded);
    void refreshSummary();
    void refreshHistory();
    int collapsedHeight() const;
    int expandedHeight() const;

private:
    QLabel* m_summaryLabel = nullptr;
    QLabel* m_promptLabel = nullptr;
    QPlainTextEdit* m_historyEdit = nullptr;
    QStringList m_history;
    QString m_prompt;
    QString m_lastMessage;
    bool m_isExpanded = false;
};
