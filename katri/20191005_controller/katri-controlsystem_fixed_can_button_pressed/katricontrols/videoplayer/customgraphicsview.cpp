#include "customgraphicsview.h"
CustomGraphicsView::CustomGraphicsView(QWidget *parent):
  QGraphicsView(parent)
{

}
CustomGraphicsView::~CustomGraphicsView()
{

}

void CustomGraphicsView::resizeEvent(QResizeEvent *) {
 /*   QRectF bounds = this->scene()->itemsBoundingRect();
    bounds.setWidth(bounds.width()*0.9);         // to tighten-up margins
    bounds.setHeight(bounds.height()*0.9); */      // same as above
    this->fitInView(this->scene()->sceneRect(), Qt::KeepAspectRatio);
//    this->centerOn(0, 0);
}

