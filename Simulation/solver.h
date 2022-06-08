#ifndef SOLVER_H
#define SOLVER_H

#include <QString>
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

struct SolverParams
{
    static constexpr double _thresholdDistance = 2; // minimum distance to object.
    static constexpr double _w_robot = 0.5; // robot width in meters.
    static constexpr double _distance_sensor_range = 10.0; // maximum range of distance sensor, in meters.
    static constexpr double _teta_goal = 75; // angle to goal point.
    static constexpr double _gamma = 0.5; // see eq (11)
};

struct Forces
{
    QVector<DistanceSensorData> repulsiveFieldData;
    QVector<DistanceSensorData> attrFieldData;
    QVector<DistanceSensorData> totalFieldData;
};

class Solver
{
public:
    Solver();
    void init(const QString& path);
    QVector<DistanceSensorData> getSensorData() const;
    Forces calculateForces();

private:
    QVector<Obstacle> enlargeObstacles(const double w_robot);
    QVector<Obstacle> findObstacles();
    void calculateObstaclesAverages(QVector<Obstacle> &obstacles);

    QVector<DistanceSensorData> calculateRepulsiveField();
    QVector<DistanceSensorData> calculateAttractiveField();
    QVector<DistanceSensorData> calculateTotalField(const QVector<DistanceSensorData>& repulsive,
                                                    const QVector<DistanceSensorData>& attractive);
private:
    QVector<DistanceSensorData> _distanceSensorData;
};

#endif // SOLVER_H
