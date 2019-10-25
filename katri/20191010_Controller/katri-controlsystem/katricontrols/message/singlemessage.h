#ifndef SINGLEMESSAGE_H
#define SINGLEMESSAGE_H

#include <QLabel>
#include "katricontrols_export.h"
enum StatusIcon{
    INFO,
    SUCCESS,
    WARNING,
    ERROR
};

class KATRICONTROLS_EXPORT SingleMessage : public QWidget
{
public:
    SingleMessage(QWidget *parent = nullptr);
    void setTextMessage(StatusIcon status);
    void setStateIcon(StatusIcon status);

private:
    QLabel *lblicon = nullptr;
    QLabel *lblmsg = nullptr;
};

#endif // SINGLEMESSAGE_H
