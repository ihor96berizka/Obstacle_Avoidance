#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCharts>

#include "solver.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void simulateDistanceSensorSlot();
    void calculateForcesSlot();
private:
    // data processing helpers
    void plotDistanceSensorData(const std::vector<Solver::DistanceSensorData>& data);
    void plotAllForces();
    //building gui functions
    void createMenus();
    void createActions();

private:
    // gui data members
    Ui::MainWindow *ui;
    QMenu *_distanceSensorMenu;
    QAction *_simulateSensorAct;
    QAction *_calculateForcesAct;
    // plotting primitives
    QChart* chart;
    QChartView* chartView;

    // data primitives
    Solver::Solver _solver;
    Solver::Forces _forces;
};
#endif // MAINWINDOW_H
