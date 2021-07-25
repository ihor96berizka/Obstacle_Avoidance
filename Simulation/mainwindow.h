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
private:
    QVector<DistanceSensorData> simulateDistanceSensor();
    void plotDistanceSensorData();
private:
    Ui::MainWindow *ui;

    // plotting primitives

    // data primitives
    QVector<DistanceSensorData> _distanceSensorData;
    static constexpr double _thresholdDistance = 2;
};
#endif // MAINWINDOW_H
