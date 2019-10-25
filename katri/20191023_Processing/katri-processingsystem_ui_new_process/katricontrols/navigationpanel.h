#ifndef NAVIGATIONPANEL_H
#define NAVIGATIONPANEL_H

#include <QWidget>
#include <QCheckBox>
#include <QRadioButton>
#include "katricontrols_export.h"

namespace Ui {
class NavigationPanel;
}

class KATRICONTROLS_EXPORT NavigationPanel : public QWidget
{
    Q_OBJECT

public:
    explicit NavigationPanel(QWidget *parent = nullptr);
    ~NavigationPanel();

    void loadToConnection(const QObject *receiver, const char *member);
    void browseConnection(const QObject *receiver, const char *member);
    void exportConnection(const QObject *receiver, const char *member);
    void enableLoadToButton(bool status);
    void enableBrowseButton(bool status);
    void enableExportButton(bool status);
    void setStyleNavCheckbox(QCheckBox *chk);
    void clickedCheckBox(QCheckBox *cbSender, const QObject *receiver, const char *member);
    void setStyleNavRadioButton(QRadioButton *rad);
    void clickedRadioButton(QRadioButton *rdSender, const QObject *receiver, const char *member);
    QList<QRadioButton*> getListObjectRadio();
    QList<QRadioButton*> getListObjectCalib();

Q_SIGNALS:
    void signalEnableAllButton(bool);

public Q_SLOTS:
    void slotEnableAllButton(bool);

private:
    Ui::NavigationPanel *ui;

};

#endif // NAVIGATIONPANEL_H
