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

struct Obstacle
{
    QVector<double> distances;
    QVector<double> angles;
    double averageDistance;
    double averageAngle;
    double a;
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
    QVector<Obstacle> enlargeObstacles(const double w_robot);
    QVector<Obstacle> findObstacles();
    void calculateObstaclesAverages(QVector<Obstacle> &obstacles);
    void calculateRepulsiveField();
    //building gui functions
    void createMenus();
    void createActions();

private:
    // gui data members
    Ui::MainWindow *ui;
    QMenu *_distanceSensorMenu;
    QAction *_simulateSensorAct;
    // plotting primitives
    QChart* chart;
    QChartView* chartView;

    // data primitives
    QVector<DistanceSensorData> _distanceSensorData;
    static constexpr double _thresholdDistance = 2; // minimum distance to object.
    static constexpr double _w_robot = 0.3; // robot width in meters.
    static constexpr double _distance_sensor_range = 5.0; // maximum range of distance sensor, in meters.
};
#endif // MAINWINDOW_H
