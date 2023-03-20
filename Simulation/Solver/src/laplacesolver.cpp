#include "laplacesolver.h"

#include <cmath>
#include <numeric>
#include <algorithm>

#include <iostream>
namespace Solver
{

int LaplaceSolver::calculateHeadingAngle()
{
    _distanceSensorData = _dataProvider->getSample();
    calculateForces();
    return std::min_element(std::begin(_forces.totalFieldData), std::end(_forces.totalFieldData),
                            [](const DistanceSensorData& lhs, const DistanceSensorData& rhs)
           {
               return lhs.distance < rhs.distance;
           })->angle;
}

std::vector<DistanceSensorData> LaplaceSolver::calculateRepulsiveField()
{
    //  find obstacles in distance sensors data.
    auto obstacles = findObstacles();

    //calculate d[k] and phi[k] - for (6)
    calculateObstaclesAverages(obstacles);

    enlargeObstacles(obstacles, SolverParams::_w_robot);

    // (9)
    for (size_t k = 0; k < obstacles.size(); ++k)
    {
        double d = SolverParams::_distance_sensor_range - (obstacles[k].averageDistance);
        obstacles[k].a =  d * std::exp(0.5);
        //qInfo() << "A[" << k << "]=" << obstacles[k].a;
    }

    // (10)
    std::vector<DistanceSensorData> repulsiveFieldData;
    for (size_t i = 0; i < _distanceSensorData.size(); ++i) // distance sensor data is used, cause it holds angles.
    {
        //qInfo() << "calculating...";
        double sum = 0;
        for (size_t k = 0; k < obstacles.size(); ++k)
        {
            int midIdx = obstacles[k].angles.size() / 2;
            double sigma = obstacles[k].averageAngle / 2.0;  // half of the angle occupied by obstacle
            std::cout << "AverageAngle: " << obstacles[k].averageAngle << std::endl;
            //qInfo() << "angle: " << obstacles[k].averageAngle;
            //qInfo() << "midIDx: " << midIdx;
            std::cout << "sigma/: " << sigma << std::endl;

            double Teta_k = obstacles[k].angles[midIdx];  //center angle of the obstacle
            std::cout << "teta[0]: " << Teta_k << std::endl;
            double underExp = -(std::sqrt(2) * std::abs(DegreesToRadians(Teta_k - _distanceSensorData[i].angle)))
                    /
                    sigma;
            //qInfo() << "A[k]: " << obstacles[k].a;
            double val = (obstacles[k].averageDistance * std::exp(std::sqrt(2))) * std::exp(underExp);
            sum += val;
        }
        //qInfo() << "angle: " << _distanceSensorData[i].angle << "val = " << sum;
        repulsiveFieldData.push_back({_distanceSensorData[i].angle, sum});
    }

    //qInfo() << "items:" << repulsiveFieldData.size();

    return repulsiveFieldData;
}

std::vector<DistanceSensorData> LaplaceSolver::calculateAttractiveField()
{
    std::vector<DistanceSensorData> attrFieldData;
    for (size_t i = 0; i < _distanceSensorData.size(); ++i) // distance sensor data is used, cause it holds angles.
    {
        double value = SolverParams::_gamma * std::abs(DegreesToRadians(SolverParams::_teta_goal - _distanceSensorData[i].angle));
        attrFieldData.push_back({_distanceSensorData[i].angle, value});
    }

    return attrFieldData;
}

}  //namespace
