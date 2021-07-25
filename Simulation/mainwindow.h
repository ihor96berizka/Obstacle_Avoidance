#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <QtCharts>

#include <QVector>

struct DistanceSensorData
{
    int angle;
    double distance;
};

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
private:
    // data processing helpers
    QVector<DistanceSensorData> simulateDistanceSensor();
    void plotDistanceSensorData();

    //building gui functions
    void createMenus();
    void createActions();
private:
    // gui data members
    Ui::MainWindow *ui;
    QMenu *_distanceSensorMenu;
    QAction *_simulateSensorAct;
    // plotting primitives

    // data primitives
    QVector<DistanceSensorData> _distanceSensorData;
    static constexpr double _thresholdDistance = 2;
};
#endif // MAINWINDOW_H
