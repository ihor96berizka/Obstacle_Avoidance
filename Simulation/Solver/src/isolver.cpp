#include "isolver.h"

#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>

namespace Solver
{

void ISolver::init(std::unique_ptr<IDataProvider> dataProvider)
{
    _dataProvider = std::move(dataProvider);
}

std::vector<DistanceSensorData> ISolver::getSensorData()
{
    _distanceSensorData = _dataProvider->getSample();
    return _distanceSensorData;
}

Forces ISolver::getForces()
{
    return _forces;
}

void ISolver::calculateForces()
{
    auto repulsive =  calculateRepulsiveField();
    auto attractive = calculateAttractiveField();
    auto total = calculateTotalField(repulsive, attractive);
    _forces = {repulsive, attractive, total};
}

std::vector<DistanceSensorData> ISolver::calculateTotalField(const std::vector<DistanceSensorData> &repulsive,
                                                            const std::vector<DistanceSensorData> &attractive)
{
    std::vector<DistanceSensorData> total;
    for (size_t idx = 0; idx < repulsive.size(); ++idx)
    {
        total.push_back({repulsive[idx].angle,
                         repulsive[idx].distance + attractive[idx].distance});
    }

    return total;
}

std::vector<Obstacle> ISolver::findObstacles()
{
    std::vector<Obstacle> obstacles;
    std::vector<DistanceSensorData> filteredDataWithObtstacles;
    // copy data from distance sensor if distance < threashold.
    // this means that obstacle was detected on that angle.
    std::copy_if(_distanceSensorData.begin(), _distanceSensorData.end(),
                 std::back_inserter(filteredDataWithObtstacles),
                 [](const auto& item)
    {
        return item.distance < SolverParams::_thresholdDistance;
    });

    for (auto& item : filteredDataWithObtstacles)
    {
        //qInfo() << "angle: " << item.angle << " | distance:" << item.distance;
    }

    // detect each obstacle and gather angles occupied by each obstacle.
    size_t obstacle_start_idx = 0;
    for (size_t idx = 0; idx < filteredDataWithObtstacles.size() - 1; ++idx)
    {
        //qInfo() << "filteredDataWithObtstacles[idx+1].angle - filteredDataWithObtstacles[idx].angle > 1 : "
         //       << (filteredDataWithObtstacles[idx+1].angle - filteredDataWithObtstacles[idx].angle > 1);
        //qInfo() <<"idx: " << idx;
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
    //qInfo() << "------obstacles detected----------";
   // qInfo() << "Number of obstacles: " << obstacles.size();
    for (size_t k = 0; k < obstacles.size(); ++k)
    {
        //qInfo() << "Obstacle # " << k;
        for (size_t i = 0; i < obstacles[k].angles.size(); ++i)
        {
            //qInfo() << "angle: " << obstacles[k].angles[i] << " | distance:" << obstacles[k].distances[i];
            //obstacles[k].angles[i] = DegreesToRadians(obstacles[k].angles[i]);
        }
    }

    return obstacles;
}

void ISolver::calculateObstaclesAverages(std::vector<Obstacle> &obstacles)
{
    for (auto& item : obstacles)
    {
        double averageDistance = std::accumulate(item.distances.begin(), item.distances.end(), 0.0) / item.distances.size();
        std::cout << "First angle: " << item.angles.at(0) << "  Last angle: " << item.angles.at(item.angles.size()-1) << std::endl;
        double averageAngle = item.angles.at(item.angles.size()-1) - item.angles.at(0);
                //std::accumulate(item.angles.begin(), item.angles.end(), 0.0) / item.angles.size();
        item.averageDistance = averageDistance;
        item.averageAngle = averageAngle;

        std::cout << "Average dist: " << averageDistance << " | avg angle: " << item.averageAngle << std::endl;
    }
}

void ISolver::enlargeObstacles(std::vector<Obstacle>& obstacles, const double w_robot)
{
    for (auto& item : obstacles)
    {
        // ata2 returns value in radians!!!
       item.averageAngle = 2 * std::atan2(item.averageDistance * std::tan(DegreesToRadians(item.averageAngle / 2.0)) + w_robot / 2.0,
                                           item.averageDistance);// * 57.2957795 + 180; // (6)
    }
    //return obstacles;
}

}  // namespace Solver
