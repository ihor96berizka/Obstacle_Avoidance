#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <random>
#include <algorithm>

#include <QJsonObject>
#include <QFile>

namespace
{
const char* kDistanceSensotFilePath{":/distance_sensor/distance_sensor_data.json"};//obstacles [20;65], [71;108]
  //  ":/distance_sensor/distance_sensor_data_test.json"}; //obstacle is on angles[71;108]


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
    delete chart;
    delete ui;
}

void MainWindow::simulateDistanceSensorSlot()
{
    _distanceSensorData = simulateDistanceSensor();
    plotDistanceSensorData();
}

void MainWindow::calculateForcesSlot()
{
    auto repulsive =  calculateRepulsiveField();
    auto attractive = calculateAttractiveField();
    auto total = calculateTotalField(repulsive, attractive);

    chart->removeAllSeries();
    QLineSeries* repulsiveSeries = new QLineSeries(chart);
    repulsiveSeries->setName("F_Rep(Teta)");
    for (auto& item : repulsive)
    {
        repulsiveSeries->append(item.angle, item.distance);
    }
    chart->addSeries(repulsiveSeries);

    QLineSeries* attractiveSeries = new QLineSeries(chart);
    attractiveSeries->setName("F_Attr(Teta)");
    for (auto& item : attractive)
    {
        attractiveSeries->append(item.angle, item.distance);
    }
    chart->addSeries(attractiveSeries);

    QLineSeries* totalSeries = new QLineSeries(chart);
    totalSeries->setName("F_Total(Teta)");
    for (auto& item : total)
    {
        totalSeries->append(item.angle, item.distance);
    }
    chart->addSeries(totalSeries);

    chart->createDefaultAxes();

    chart->legend()->setVisible(true);
    chart->legend()->setAlignment(Qt::AlignBottom);
    chart->axes(Qt::Horizontal).first()->setTitleText("angle");
    QAbstractAxis* yAxis =  chart->axes(Qt::Vertical).first();
    yAxis->setTitleText("F_x(Teta[i])");
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
    threasholdLine->setName("Threashold distance");
    distanceSeries->setName("distance to obstacle");

    for (auto& item : _distanceSensorData)
    {
        threasholdLine->append(item.angle, _thresholdDistance);
        distanceSeries->append(item.angle, item.distance);
    }

    chart = new QChart;
    chart->addSeries(threasholdLine);
    chart->addSeries(distanceSeries);

    chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);
    chart->createDefaultAxes();

    chart->legend()->setVisible(true);
    chart->legend()->setAlignment(Qt::AlignBottom);
    chart->axes(Qt::Horizontal).first()->setTitleText("angle");
    QAbstractAxis* yAxis =  chart->axes(Qt::Vertical).first();
    yAxis->setTitleText("distance, m");

    this->setCentralWidget(chartView);
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
        double averageAngle = item.angles.last() - item.angles.first();
                //std::accumulate(item.angles.begin(), item.angles.end(), 0.0) / item.angles.size();
        item.averageDistance = averageDistance;
        item.averageAngle = averageAngle;

        qInfo() << "Average dist: " << averageDistance << " | avg angle: " << item.averageAngle;
    }
}

QVector<DistanceSensorData> MainWindow::calculateRepulsiveField()
{
    auto obstacles = enlargeObstacles(_w_robot);

    // (9)
    for (int k = 0; k < obstacles.size(); ++k)
    {
        double d = _distance_sensor_range - (obstacles[k].averageDistance);
        obstacles[k].a =  d * std::exp(0.5);
        qInfo() << "A[" << k << "]=" << obstacles[k].a;
    }

    // (10)
    QVector<DistanceSensorData> repulsiveFieldData;
    for (int i = 0; i < _distanceSensorData.size(); ++i) // distance sensor data is used, cause it holds angles.
    {
        //qInfo() << "calculating...";
        double sum = 0;
        for (int k = 0; k < obstacles.size(); ++k)
        {
            int midIdx = obstacles[k].angles.size() / 2;
            double sigma = obstacles[k].averageAngle / 2.0;  // half of the angle occupied by obstacle
            //qInfo() << "angle: " << obstacles[k].averageAngle;
            //qInfo() << "midIDx: " << midIdx;
            qInfo() << "sigma/: " << sigma;

            double Teta_k = obstacles[k].angles[midIdx];  //center angle of the obstacle
            //qInfo() << "teta[0]: " << Teta_k;
            double underExp = -(std::pow(Teta_k - _distanceSensorData[i].angle, 2))
                    /
                    2.0 * std::pow(sigma, 2);
            //qInfo() << "A[k]: " << obstacles[k].a;
            double val = obstacles[k].a * std::exp(underExp);
            sum += val;
        }
        qInfo() << "angle: " << _distanceSensorData[i].angle << "val = " << sum;
        repulsiveFieldData.push_back({_distanceSensorData[i].angle, sum});
    }

    qInfo() << "items:" << repulsiveFieldData.size();

    return repulsiveFieldData;
}

QVector<DistanceSensorData> MainWindow::calculateAttractiveField()
{
    QVector<DistanceSensorData> attrFieldData;
    for (int i = 0; i < _distanceSensorData.size(); ++i) // distance sensor data is used, cause it holds angles.
    {
        double value = _gamma * abs(_teta_goal - _distanceSensorData[i].angle);
        attrFieldData.push_back({_distanceSensorData[i].angle, value});
    }

    return attrFieldData;
}

QVector<DistanceSensorData> MainWindow::calculateTotalField(const QVector<DistanceSensorData> &repulsive,
                                                            const QVector<DistanceSensorData> &attractive)
{
    QVector<DistanceSensorData> total;
    for (int idx = 0; idx < repulsive.size(); ++idx)
    {
        total.push_back({repulsive[idx].angle,
                         repulsive[idx].distance + attractive[idx].distance});
    }

    return total;
}

void MainWindow::createMenus()
{
    _distanceSensorMenu = menuBar()->addMenu(tr("Distance sensor"));
    _distanceSensorMenu->addAction(_simulateSensorAct);
    _distanceSensorMenu->addAction(_calculateForcesAct);
}

void MainWindow::createActions()
{
    _simulateSensorAct = new QAction(tr("Simulate data"), this);
    _simulateSensorAct->setToolTip(tr("Reads distance data from json file"));
    connect(_simulateSensorAct, &QAction::triggered, this, &MainWindow::simulateDistanceSensorSlot);

    _calculateForcesAct = new QAction(tr("Calculate forces"));
    _calculateForcesAct->setToolTip(tr("Calculate f_rep, F_attr and F_total"));
    connect(_calculateForcesAct, &QAction::triggered, this, &MainWindow::calculateForcesSlot);
}

