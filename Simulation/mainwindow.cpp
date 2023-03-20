#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QJsonObject>
#include <QFile>

#include "jsondataprovider.h"
#include "solver.h"
#include "linearsolver.h"
#include "laplacesolver.h"

namespace
{
const char* kDistanceSensotFilePath{":/distance_sensor/distance_sensor_data.json"};//obstacles [20;65], [71;108]
  //  ":/distance_sensor/distance_sensor_data_test.json"}; //obstacle is on angles[71;108]
} // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    createActions();
    createMenus();
    auto dataproviderPtr = std::make_unique<JsonDataProvider>(kDistanceSensotFilePath);
    _solver = //std::make_unique<Solver::LinearSolver>();
            std::make_unique<Solver::GussianSolver>();
            //std::make_unique<Solver::LaplaceSolver>();
    _solver->init(std::move(dataproviderPtr));
}

MainWindow::~MainWindow()
{
    delete chart;
    delete ui;
}

void MainWindow::simulateDistanceSensorSlot()
{
    auto distanceSensorData = _solver->getSensorData();
    plotDistanceSensorData(distanceSensorData);
}

void MainWindow::calculateForcesSlot()
{
    int angle = _solver->calculateHeadingAngle();
    _forces = _solver->getForces();
    qInfo() << "-----Angle: " << (angle);
    plotAllForces();
}

void MainWindow::plotDistanceSensorData(const std::vector<Solver::DistanceSensorData> &data)
{
    QLineSeries* threasholdLine = new QLineSeries;
    QLineSeries* distanceSeries = new QLineSeries;
    threasholdLine->setName("Threashold distance");
    distanceSeries->setName("distance to obstacle");

    for (auto& item : data)
    {
        threasholdLine->append(/*Solver::RadiansToDegrees*/(item.angle), Solver::SolverParams::_thresholdDistance);
        distanceSeries->append(/*Solver::RadiansToDegrees*/(item.angle), item.distance);
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

void MainWindow::plotAllForces()
{
    auto [repulsive, attractive, total] = _forces;
    chart->removeAllSeries();
    QLineSeries* repulsiveSeries = new QLineSeries(chart);
    repulsiveSeries->setName("F_Rep(Teta)");
    for (auto& item : repulsive)
    {
        repulsiveSeries->append(/*Solver::RadiansToDegrees*/(item.angle), item.distance);
    }
    chart->addSeries(repulsiveSeries);
    QLineSeries* attractiveSeries = new QLineSeries(chart);
    attractiveSeries->setName("F_Attr(Teta)");
    for (auto& item : attractive)
    {
        attractiveSeries->append(/*Solver::RadiansToDegrees*/(item.angle), item.distance);
    }
    chart->addSeries(attractiveSeries);
    QLineSeries* totalSeries = new QLineSeries(chart);
    totalSeries->setName("F_Total(Teta)");
    for (auto& item : total)
    {
        totalSeries->append(/*Solver::RadiansToDegrees*/(item.angle), item.distance);
    }
    chart->addSeries(totalSeries);
    chart->createDefaultAxes();
    chart->legend()->setVisible(true);
    chart->legend()->setAlignment(Qt::AlignBottom);
    chart->axes(Qt::Horizontal).first()->setTitleText("angle");
    QAbstractAxis* yAxis =  chart->axes(Qt::Vertical).first();
    yAxis->setTitleText("F_x(Teta[i])");
}

void MainWindow::createMenus()
{
    _distanceSensorMenu = menuBar()->addMenu(tr("Distance sensor"));
    _distanceSensorMenu->addAction(_simulateSensorAct);
    _distanceSensorMenu->addAction(_calculateGaussianForcesAct);
}

void MainWindow::createActions()
{
    _simulateSensorAct = new QAction(tr("Simulate data"), this);
    _simulateSensorAct->setToolTip(tr("Reads distance data from json file"));
    connect(_simulateSensorAct, &QAction::triggered, this, &MainWindow::simulateDistanceSensorSlot);

    _calculateGaussianForcesAct = new QAction(tr("Calculate Gaussian forces"));
    _calculateGaussianForcesAct->setToolTip(tr("Calculate f_rep, F_attr and F_total"));
    connect(_calculateGaussianForcesAct, &QAction::triggered, this, &MainWindow::calculateForcesSlot);
}

