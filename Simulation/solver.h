#ifndef SOLVER_H
#define SOLVER_H

#include <string>
#include <vector>

struct DistanceSensorData
{
    int angle;
    double distance;
};

struct Obstacle
{
    std::vector<double> distances;
    std::vector<double> angles;
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
    std::vector<DistanceSensorData> repulsiveFieldData;
    std::vector<DistanceSensorData> attrFieldData;
    std::vector<DistanceSensorData> totalFieldData;
};

class Solver
{
public:
    Solver();
    void init(const std::string& path);
    std::vector<DistanceSensorData> getSensorData() const;
    Forces calculateForces();
    std::vector<std::vector<DistanceSensorData>> getRepulsiceComponents();

private:
    std::vector<Obstacle> enlargeObstacles(const double w_robot);
    std::vector<Obstacle> findObstacles();
    void calculateObstaclesAverages(std::vector<Obstacle> &obstacles);

    std::vector<DistanceSensorData> calculateRepulsiveField();
    std::vector<DistanceSensorData> calculateAttractiveField();
    std::vector<DistanceSensorData> calculateTotalField(const std::vector<DistanceSensorData>& repulsive,
                                                        const std::vector<DistanceSensorData>& attractive);
private:
    std::vector<DistanceSensorData> _distanceSensorData;
};

#endif // SOLVER_H
