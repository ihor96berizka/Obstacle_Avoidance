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

    threasholdLine->setName("Threashold distance");
    distanceSeries->setName("distance to obstacle");

    for (auto& item : _distanceSensorData)
    {
        threasholdLine->append(item.angle, _thresholdDistance);
        distanceSeries->append(item.angle, item.distance);
    }

    QChart* chart = new QChart;
    chart->addSeries(threasholdLine);
    chart->addSeries(distanceSeries);


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

