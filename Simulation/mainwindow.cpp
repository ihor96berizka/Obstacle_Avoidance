#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <random>
#include <algorithm>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    _distanceSensorData = simulateDistanceSensor();
    plotDistanceSensorData();
}

MainWindow::~MainWindow()
{
    delete ui;
}

QVector<DistanceSensorData> MainWindow::simulateDistanceSensor()
{
    return {};
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
    chart->axes(Qt::Vertical).first()->setTitleText("distance, m");
    this->setCentralWidget(chartView);
}

