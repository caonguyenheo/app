#ifndef CUSTOMGRAPHICSVIEW_H
#define CUSTOMGRAPHICSVIEW_H
#include <QWidget>
#include <QGraphicsView>
#include "katricontrols_export.h"

class KATRICONTROLS_EXPORT CustomGraphicsView : public QGraphicsView
{
    Q_OBJECT
public:
    explicit CustomGraphicsView(QWidget *parent = nullptr);
    ~CustomGraphicsView();
public:
    explicit CustomGraphicsView();
    void resizeEvent(QResizeEvent *);
};

#endif // CUSTOMGRAPHICSVIEW_H
