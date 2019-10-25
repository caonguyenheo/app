#ifndef SIMULATIONVIEW_H
#define SIMULATIONVIEW_H

#include <QWidget>
#include <QLabel>
#include <QMovie>

namespace Ui {
class SimulationView;
}

class SimulationView : public QWidget
{
    Q_OBJECT

public:
    explicit SimulationView(QWidget *parent = nullptr);
    ~SimulationView();

    void setViewName(QString &name);
    void setViewColor(QColor &color);
    void appendStyleSheet(QString stylesheet);
    void setImageSimulation(bool display);
    void setStyleMainScreenLayout();
    void setStyleFullScreenLayout();
    void setImage(QLabel *image);
protected:
    void resizeEvent(QResizeEvent *event) override;

private:
    Ui::SimulationView *ui;
    QLabel *lbicon = nullptr;
    QMovie *movie = nullptr;
};

#endif // SIMULATIONVIEW_H
