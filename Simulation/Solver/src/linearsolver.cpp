#include "linearsolver.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace Solver
{

int LinearSolver::calculateHeadingAngle()
{
    _distanceSensorData = _dataProvider->getSample();
    calculateForces();
    return std::min_element(std::begin(_forces.totalFieldData), std::end(_forces.totalFieldData),
                            [](const DistanceSensorData& lhs, const DistanceSensorData& rhs)
           {
               return lhs.distance < rhs.distance;
           })->angle;
}

std::vector<Solver::DistanceSensorData> LinearSolver::calculateRepulsiveField()
{
    std::vector obstacles = findObstacles();

    //calculate d[k] and phi[k] - for (6)
    calculateObstaclesAverages(obstacles);

    std::vector<DistanceSensorData> repulsiveFieldData;
    for (size_t i = 0; i < _distanceSensorData.size(); ++i) // distance sensor data is used, cause it holds angles.
    {
        //qInfo() << "calculating...";
        double sum = 0;
        for (size_t k = 0; k < obstacles.size(); ++k)
        {
            double val =  (SolverParams::_distance_sensor_range - _distanceSensorData[i].distance) * obstacles[k].averageDistance;
            sum += val;
        }
        //qInfo() << "angle: " << _distanceSensorData[i].angle << "val = " << sum;
        repulsiveFieldData.push_back({_distanceSensorData[i].angle, sum});
    }


    //qInfo() << "items:" << repulsiveFieldData.size();

    return repulsiveFieldData;
}

std::vector<Solver::DistanceSensorData> LinearSolver::calculateAttractiveField()
{
    std::vector<DistanceSensorData> attrFieldData;
    for (size_t i = 0; i < _distanceSensorData.size(); ++i) // distance sensor data is used, cause it holds angles.
    {
        double value = SolverParams::_gamma * 0.8 * fabs(SolverParams::_teta_goal - _distanceSensorData[i].angle);
        attrFieldData.push_back({_distanceSensorData[i].angle, value});
    }

    return attrFieldData;
}

} // namespace Solver
