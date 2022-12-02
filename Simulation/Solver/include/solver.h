#ifndef SOLVER_H
#define SOLVER_H

#include <memory>
#include <string>
#include <vector>

#include "idataprovider.h"

namespace Solver
{

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
/*
* Usage: Create instance of Solver.
* Flow:
*   init(provider)
*   calculateHeadingAngle()
* calculateHeadingAngle() will wait for new chuck of data and then perform calculations.
* It should be called in a working loop in user code.
*/
class Solver
{
public:
    void init(std::unique_ptr<IDataProvider> dataProvider);
    std::vector<DistanceSensorData> getSensorData();
    Forces getForces();
    int calculateHeadingAngle();

private:
    std::vector<Obstacle> enlargeObstacles(const double w_robot);
    std::vector<Obstacle> findObstacles();
    void calculateForces();
    void calculateObstaclesAverages(std::vector<Obstacle> &obstacles);
    std::vector<std::vector<DistanceSensorData>> getRepulsiceComponents();
    std::vector<DistanceSensorData> calculateRepulsiveField();
    std::vector<DistanceSensorData> calculateAttractiveField();
    std::vector<DistanceSensorData> calculateTotalField(const std::vector<DistanceSensorData>& repulsive,
                                                        const std::vector<DistanceSensorData>& attractive);
private:
    std::vector<DistanceSensorData> _distanceSensorData;
    Forces _forces;
    std::unique_ptr<IDataProvider> _dataProvider;
};

} //namespace Solver
#endif // SOLVER_H
