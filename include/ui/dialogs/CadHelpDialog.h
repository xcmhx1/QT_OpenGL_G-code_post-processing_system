// 声明 CadHelpDialog 模块，对外提供分类帮助与教程入口。
#pragma once

#include <QDialog>
#include <QString>

class QTabWidget;
class QTextBrowser;

enum class CadHelpSection
{
    QuickStart,
    Shortcuts,
    Drawing,
    Editing,
    Machining,
    BitmapImport,
    Appearance,
    About
};

class CadHelpDialog : public QDialog
{
public:
    explicit CadHelpDialog(QWidget* parent = nullptr);

    void setCurrentSection(CadHelpSection section);

private:
    QTextBrowser* addSection(CadHelpSection section, const QString& title, const QString& htmlBody);

private:
    QTabWidget* m_tabWidget = nullptr;
};
