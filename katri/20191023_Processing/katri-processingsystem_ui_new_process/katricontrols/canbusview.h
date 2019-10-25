#ifndef CANBUSVIEW_H
#define CANBUSVIEW_H

#include <QWidget>
#include <QLabel>
#include <QMovie>
#include <QQuickWidget>
#include <QtDataVisualization>
#include <QtWidgets/QWidget>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFontComboBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMessageBox>
#include <QtGui/QScreen>
#include <QtGui/QFontDatabase>

namespace Ui {
class CANBusView;
}

class CANBusView : public QWidget
{
    Q_OBJECT

public:
    explicit CANBusView(QWidget *parent = nullptr);
    ~CANBusView() override;

    void setCanLayout();
    void setViewName(QString &name);
    void setViewColor(QColor &color);
    void setStyleMainScreenLayout();
    void setStyleFullScreenLayout();
    void appendStyleSheet(QString stylesheet);
    void setImageSimulation(bool display);
    QString appendDataToCache(QString value);

protected:
    void resizeEvent(QResizeEvent *event) override;

Q_SIGNALS:
    void signalFinishedRender();

public Q_SLOTS:
    void slotReceiveTimestamp(int,QString);

private:
    Ui::CANBusView *ui;
    QLabel      *lbicon = nullptr;
    QMovie      *movie = nullptr;
    QLabel      *CANBusLabel;
    QHBoxLayout *hLayout;
    QVBoxLayout *vLayout1;

    QStringList *m_dataCache;
    int m_dataRowCount;
};

#endif // CANBUSVIEW_H
