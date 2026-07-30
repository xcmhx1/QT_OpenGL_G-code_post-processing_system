// 声明 CadHelpDialog 模块，对外提供分类帮助与教程入口。
#pragma once

#include <QDialog>
#include <QElapsedTimer>
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
    RotaryMachining,
    BitmapImport,
    Appearance,
    Troubleshooting,
    About
};

class CadHelpDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CadHelpDialog(QWidget* parent = nullptr);

    void setCurrentSection(CadHelpSection section);
    void setAboutInformation
    (
        const QString& aboutText,
        const QString& companyName,
        const QString& website,
        const QString& supportText,
        const QString& licenseText
    );
    void refreshAboutContent();

signals:
    void displayTitleConfigurationRequested();

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    QTextBrowser* addSection(CadHelpSection section, const QString& title, const QString& htmlBody);
    void resetAboutClickSequence();

private:
    QTabWidget* m_tabWidget = nullptr;
    QTextBrowser* m_aboutBrowser = nullptr;
    QElapsedTimer m_aboutClickTimer;
    int m_aboutClickCount = 0;
    QString m_aboutText;
    QString m_companyName;
    QString m_website;
    QString m_supportText;
    QString m_licenseText;
};
