#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <random>
#include <algorithm>

#include <QJsonObject>
#include <QFile>

namespace
{
const char* kDistanceSensotFilePath{":/distance_sensor/distance_sensor_data.json"};

const char* kDataKey{"Data"};
const char* kAngleKey{"angle"};
const char* kDistanceKey{"distance"};
} // namespace
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    createActions();
    createMenus();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::simulateDistanceSensorSlot()
{
    _distanceSensorData = simulateDistanceSensor();
    plotDistanceSensorData();
}

QVector<DistanceSensorData> MainWindow::simulateDistanceSensor()
{
    QVector<DistanceSensorData> parsedData;
    QFile file(kDistanceSensotFilePath);

    if (!file.open(QIODevice::ReadOnly))
    {
        qWarning() << "failed to open file:" << file.errorString();
        return {};
    }

    QByteArray rawData = file.readAll();

    QJsonObject jsonObj = QJsonDocument::fromJson(rawData).object();
    if (jsonObj.contains(kDataKey) && jsonObj[kDataKey].isArray())
    {
        qInfo() << "parsing json...";
        QJsonArray array = jsonObj[kDataKey].toArray();

        for (int idx = 0; idx < array.size(); ++idx)
        {
            parsedData.append({array[idx].toObject()[kAngleKey].toInt(),
                               array[idx].toObject()[kDistanceKey].toDouble()});
        }
    }
    qInfo() << "Parsing done.";

    return parsedData;
}

void MainWindow::plotDistanceSensorData()
{
    QLineSeries* threasholdLine = new QLineSeries;
    QLineSeries* distanceSeries = new QLineSeries;
    //QBarSeries* distabceBarSeries = new QBarSeries;
    QScatterSeries *distanceScatterSeries = new QScatterSeries;
    threasholdLine->setName("Threashold distance");
    //distanceSeries->setName("distance to obstacle");

    for (auto& item : _distanceSensorData)
    {
        threasholdLine->append(item.angle, _thresholdDistance);
        distanceScatterSeries->append(item.angle, item.distance);
        distanceSeries->append(item.angle, item.distance);
        //QBarSet *bar = new QBarSet(QString::number(item.angle));
        //bar->append(item.distance);
        //distabceBarSeries->append(bar);
    }

    QChart* chart = new QChart;
    chart->addSeries(threasholdLine);
    chart->addSeries(distanceSeries);
    //chart->addSeries(distabceBarSeries);
    chart->addSeries(distanceScatterSeries);

    QChartView* chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);
    chart->createDefaultAxes();

    chart->legend()->setVisible(true);
    chart->legend()->setAlignment(Qt::AlignBottom);
    chart->axes(Qt::Horizontal).first()->setTitleText("angle");
    QAbstractAxis* yAxis =  chart->axes(Qt::Vertical).first();
    yAxis->setTitleText("distance, m");
    yAxis->setMin(0);
    this->setCentralWidget(chartView);

    calculateRepulsiveField();
}

QVector<Obstacle> MainWindow::enlargeObstacles(const double w_robot)
{
    //  find obstacles in distance sensors data.
    auto obstacles = findObstacles();

    //calculate d[k] and phi[k] - for (6)
    calculateObstaclesAverages(obstacles);

    for (auto& item : obstacles)
    {
        item.averageAngle = 2 * std::atan2(item.averageDistance * std::tan(item.averageAngle / 2.0) + w_robot / 2.0,
                                           item.averageDistance); // (6)
    }
    return obstacles;
}

QVector<Obstacle> MainWindow::findObstacles()
{
    QVector<Obstacle> obstacles;
    QVector<DistanceSensorData> filteredDataWithObtstacles;
    // copy data from distance sensor if distance < threashold.
    // this means that obstacle was detected on that angle.
    std::copy_if(_distanceSensorData.begin(), _distanceSensorData.end(),
                 std::back_inserter(filteredDataWithObtstacles),
                 [](const auto& item)
    {
        return item.distance < _thresholdDistance;
    });

    for (auto& item : filteredDataWithObtstacles)
    {
        qInfo() << "angle: " << item.angle << " | distance:" << item.distance;
    }

    // detect each obstacle and gather angles occupied by each obstacle.
    int obstacle_start_idx = 0;
    for (int idx = 0; idx < filteredDataWithObtstacles.size() - 1; ++idx)
    {
        qInfo() << "filteredDataWithObtstacles[idx+1].angle - filteredDataWithObtstacles[idx].angle > 1 : "
                << (filteredDataWithObtstacles[idx+1].angle - filteredDataWithObtstacles[idx].angle > 1);
        qInfo() <<"idx: " << idx;
        if (filteredDataWithObtstacles[idx+1].angle - filteredDataWithObtstacles[idx].angle > 1
            ||
            idx + 1 == filteredDataWithObtstacles.size() - 1 // check if this is last item in vector
            )
        {
            int obstacle_end_idx = idx;
            // this is last angle occupied by current obstacle.
            // create Obstacle obj and push to vector
            //special handling of last item(obstacle)
            if (idx + 1 == filteredDataWithObtstacles.size()-1)
            {
                obstacle_end_idx = idx + 1;
            }
            Obstacle obstacle;
            for (int k = obstacle_start_idx; k <= obstacle_end_idx; ++k)
            {
                obstacle.angles.push_back(filteredDataWithObtstacles[k].angle);
                obstacle.distances.push_back(filteredDataWithObtstacles[k].distance);
            }
            obstacles.push_back(obstacle);
            obstacle_start_idx = idx + 1;
        }
    }
    qInfo() << "------obstacles detected----------";
    qInfo() << "Number of obstacles: " << obstacles.size();
    for (int k = 0; k < obstacles.size(); ++k)
    {
        qInfo() << "Obstacle # " << k;
        for (int i = 0; i < obstacles[k].angles.size(); ++i)
        {
            qInfo() << "angle: " << obstacles[k].angles[i] << " | distance:" << obstacles[k].distances[i];
        }
    }

    return obstacles;
}

void MainWindow::calculateObstaclesAverages(QVector<Obstacle> &obstacles)
{
    for (auto& item : obstacles)
    {
        double averageDistance = std::accumulate(item.distances.begin(), item.distances.end(), 0.0) / item.distances.size();
        double averageAngle = std::accumulate(item.angles.begin(), item.angles.end(), 0.0) / item.angles.size();
        item.averageDistance = averageDistance;
        item.averageAngle = averageAngle;

        qInfo() << "Average dist: " << averageDistance << " | avg angle: " << averageAngle;
    }
}

void MainWindow::calculateRepulsiveField()
{
    auto obstacles = enlargeObstacles(_w_robot);

    // (9)
    for (int k = 0; k < obstacles.size(); ++k)
    {
        obstacles[k].a = (_distance_sensor_range - obstacles[k].averageDistance * _distance_sensor_range) * std::exp(0.5);
    }

    // (10)

}

void MainWindow::createMenus()
{
    _distanceSensorMenu = menuBar()->addMenu(tr("Distance sensor"));
    _distanceSensorMenu->addAction(_simulateSensorAct);
}

void MainWindow::createActions()
{
    _simulateSensorAct = new QAction(tr("Simulate data"), this);
    _simulateSensorAct->setToolTip(tr("Reads distance data from json file"));
    connect(_simulateSensorAct, &QAction::triggered, this, &MainWindow::simulateDistanceSensorSlot);
}

