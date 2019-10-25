#ifndef BACKPANEL_H
#define BACKPANEL_H

#include <QWidget>
#include <QPushButton>
#include "katricontrols_export.h"

namespace Ui {
class BackPanel;
}

class KATRICONTROLS_EXPORT BackPanel : public QWidget
{
    Q_OBJECT

public:
    explicit BackPanel(QWidget *parent = nullptr);
    ~BackPanel();
    void backConnection(const QObject *receiver, const char *member);

    void setDoublePanel();
    QPushButton *backButton();

private:
    Ui::BackPanel *ui;
};

#endif // BACKPANEL_H
