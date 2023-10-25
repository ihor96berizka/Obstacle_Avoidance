#include "solver.h"

#include <cmath>
#include <numeric>
#include <algorithm>
#include <iostream>

namespace Solver
{

std::vector<std::vector<DistanceSensorData> > GussianSolver::getRepulsiceComponents()
{
    /*auto obstacles = enlargeObstacles(SolverParams::_w_robot);
    std::vector<std::vector<DistanceSensorData>> components(obstacles.size());

    // (9)
    for (size_t k = 0; k < obstacles.size(); ++k)
    {
        double d = SolverParams::_distance_sensor_range - (obstacles[k].averageDistance);
        obstacles[k].a =  d * std::exp(0.5);
        //qInfo() << "A[" << k << "]=" << obstacles[k].a;
    }

    // (10)
    for (size_t i = 0; i < _distanceSensorData.size(); ++i) // distance sensor data is used, cause it holds angles.
    {
        for (size_t k = 0; k < obstacles.size(); ++k)
        {
            int midIdx = obstacles[k].angles.size() / 2;
            double sigma = obstacles[k].averageAngle / 2.0;  // half of the angle occupied by obstacle
            //qInfo() << "angle: " << obstacles[k].averageAngle;
            //qInfo() << "midIDx: " << midIdx;
            //qInfo() << "sigma/: " << sigma;

            double Teta_k = obstacles[k].angles[midIdx];  //center angle of the obstacle
            //qInfo() << "teta[0]: " << Teta_k;
            double underExp = -(std::pow(Teta_k - _distanceSensorData[i].angle, 2))
                    /
                    2.0 * std::pow(sigma, 2);
            //qInfo() << "A[k]: " << obstacles[k].a;
            double val = obstacles[k].a * std::exp(underExp);
            components[k].push_back({_distanceSensorData[i].angle, val});
        }
    }

    return components;*/
}

int GussianSolver::calculateHeadingAngle()
{
    _distanceSensorData = _dataProvider->getSample();
    calculateForces();
    return (std::min_element(std::begin(_forces.totalFieldData), std::end(_forces.totalFieldData),
                            [](const DistanceSensorData& lhs, const DistanceSensorData& rhs)
           {
               return lhs.distance < rhs.distance;
           })->angle);
}


std::vector<DistanceSensorData> GussianSolver::calculateRepulsiveField()
{
    //  find obstacles in distance sensors data.
    auto obstacles = findObstacles();

    //calculate d[k] and phi[k] - for (6)
    calculateObstaclesAverages(obstacles);

    // (6) Phi[k] in rads
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
            std::cout << "Average angle g: " << obstacles[k].averageAngle << std::endl;
            double sigma = (obstacles[k].averageAngle / 2.0);  // half of the angle occupied by obstacle
            //qInfo() << "angle: " << obstacles[k].averageAngle;
            //qInfo() << "midIDx: " << midIdx;
            std::cout << "sigma/: " << sigma << std::endl;

            double Teta_k = (obstacles[k].angles[midIdx]);  //center angle of the obstacle
            //qInfo() << "teta[0]: " << Teta_k;
            double underExp = (std::pow((Teta_k - (_distanceSensorData[i].angle)), 2))
                    /
                    2.0 * std::pow(sigma, 2);
            //qInfo() << "A[k]: " << obstacles[k].a;
            double val = obstacles[k].a * std::exp(-underExp);
            sum += val;
        }
        //qInfo() << "angle: " << _distanceSensorData[i].angle << "val = " << sum;
        repulsiveFieldData.push_back({_distanceSensorData[i].angle, sum});
    }

    //qInfo() << "items:" << repulsiveFieldData.size();

    return repulsiveFieldData;
}

std::vector<DistanceSensorData> GussianSolver::calculateAttractiveField()
{
    std::vector<DistanceSensorData> attrFieldData;
    for (size_t i = 0; i < _distanceSensorData.size(); ++i) // distance sensor data is used, cause it holds angles.
    {
        double value = SolverParams::_gamma * std::abs((SolverParams::_teta_goal - _distanceSensorData[i].angle));
        attrFieldData.push_back({_distanceSensorData[i].angle, value});
    }

    return attrFieldData;
}

}  //namespace
