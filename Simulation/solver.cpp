#include "solver.h"

#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>

#include <QFile>

namespace
{
const char* kDataKey{"Data"};
const char* kAngleKey{"angle"};
const char* kDistanceKey{"distance"};
}
Solver::Solver()
{

}

void Solver::init(const std::string &path)
{
    _distanceSensorData.clear();
    QFile file(path.c_str());

    if (!file.open(QIODevice::ReadOnly))
    {
        qWarning() << "failed to open file:" << file.errorString();
        return;
    }

    QByteArray rawData = file.readAll();

    QJsonObject jsonObj = QJsonDocument::fromJson(rawData).object();
    if (jsonObj.contains(kDataKey) && jsonObj[kDataKey].isArray())
    {
        qInfo() << "parsing json...";
        QJsonArray array = jsonObj[kDataKey].toArray();

        for (int idx = 0; idx < array.size(); ++idx)
        {
            _distanceSensorData.push_back({array[idx].toObject()[kAngleKey].toInt(),
                               array[idx].toObject()[kDistanceKey].toDouble()});
        }
    }
    qInfo() << "Parsing done.";
}

std::vector<DistanceSensorData> Solver::getSensorData() const
{
    return _distanceSensorData;
}

Forces Solver::calculateForces()
{
    auto repulsive =  calculateRepulsiveField();
    auto attractive = calculateAttractiveField();
    auto total = calculateTotalField(repulsive, attractive);

    return {repulsive, attractive, total};
}

std::vector<std::vector<DistanceSensorData> > Solver::getRepulsiceComponents()
{
    auto obstacles = enlargeObstacles(SolverParams::_w_robot);
    std::vector<std::vector<DistanceSensorData>> components(obstacles.size());

    // (9)
    for (size_t k = 0; k < obstacles.size(); ++k)
    {
        double d = SolverParams::_distance_sensor_range - (obstacles[k].averageDistance);
        obstacles[k].a =  d * std::exp(0.5);
        qInfo() << "A[" << k << "]=" << obstacles[k].a;
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
            qInfo() << "sigma/: " << sigma;

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

    return components;
}

std::vector<Obstacle> Solver::enlargeObstacles(const double w_robot)
{
    //  find obstacles in distance sensors data.
    auto obstacles = findObstacles();

    //calculate d[k] and phi[k] - for (6)
    calculateObstaclesAverages(obstacles);

    for (auto& item : obstacles)
    {
        item.averageAngle = 2 * std::atan2(item.averageDistance * std::tan(item.averageAngle / 2.0) + w_robot / 2.0,
                                           item.averageDistance); // (6)
    }
    return obstacles;
}

std::vector<Obstacle> Solver::findObstacles()
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
        qInfo() << "angle: " << item.angle << " | distance:" << item.distance;
    }

    // detect each obstacle and gather angles occupied by each obstacle.
    size_t obstacle_start_idx = 0;
    for (size_t idx = 0; idx < filteredDataWithObtstacles.size() - 1; ++idx)
    {
        qInfo() << "filteredDataWithObtstacles[idx+1].angle - filteredDataWithObtstacles[idx].angle > 1 : "
                << (filteredDataWithObtstacles[idx+1].angle - filteredDataWithObtstacles[idx].angle > 1);
        qInfo() <<"idx: " << idx;
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
    qInfo() << "------obstacles detected----------";
    qInfo() << "Number of obstacles: " << obstacles.size();
    for (size_t k = 0; k < obstacles.size(); ++k)
    {
        qInfo() << "Obstacle # " << k;
        for (size_t i = 0; i < obstacles[k].angles.size(); ++i)
        {
            qInfo() << "angle: " << obstacles[k].angles[i] << " | distance:" << obstacles[k].distances[i];
        }
    }

    return obstacles;
}

void Solver::calculateObstaclesAverages(std::vector<Obstacle> &obstacles)
{
    for (auto& item : obstacles)
    {
        double averageDistance = std::accumulate(item.distances.begin(), item.distances.end(), 0.0) / item.distances.size();
        double averageAngle = item.angles.at(item.angles.size()-1) - item.angles.at(0);
                //std::accumulate(item.angles.begin(), item.angles.end(), 0.0) / item.angles.size();
        item.averageDistance = averageDistance;
        item.averageAngle = averageAngle;

        qInfo() << "Average dist: " << averageDistance << " | avg angle: " << item.averageAngle;
    }
}

std::vector<DistanceSensorData> Solver::calculateRepulsiveField()
{
    auto obstacles = enlargeObstacles(SolverParams::_w_robot);

    // (9)
    for (size_t k = 0; k < obstacles.size(); ++k)
    {
        double d = SolverParams::_distance_sensor_range - (obstacles[k].averageDistance);
        obstacles[k].a =  d * std::exp(0.5);
        qInfo() << "A[" << k << "]=" << obstacles[k].a;
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
            //qInfo() << "angle: " << obstacles[k].averageAngle;
            //qInfo() << "midIDx: " << midIdx;
            qInfo() << "sigma/: " << sigma;

            double Teta_k = obstacles[k].angles[midIdx];  //center angle of the obstacle
            //qInfo() << "teta[0]: " << Teta_k;
            double underExp = -(std::pow(Teta_k - _distanceSensorData[i].angle, 2))
                    /
                    2.0 * std::pow(sigma, 2);
            //qInfo() << "A[k]: " << obstacles[k].a;
            double val = obstacles[k].a * std::exp(underExp);
            sum += val;
        }
        qInfo() << "angle: " << _distanceSensorData[i].angle << "val = " << sum;
        repulsiveFieldData.push_back({_distanceSensorData[i].angle, sum});
    }

    qInfo() << "items:" << repulsiveFieldData.size();

    return repulsiveFieldData;
}

std::vector<DistanceSensorData> Solver::calculateAttractiveField()
{
    std::vector<DistanceSensorData> attrFieldData;
    for (size_t i = 0; i < _distanceSensorData.size(); ++i) // distance sensor data is used, cause it holds angles.
    {
        double value = SolverParams::_gamma * abs(SolverParams::_teta_goal - _distanceSensorData[i].angle);
        attrFieldData.push_back({_distanceSensorData[i].angle, value});
    }

    return attrFieldData;
}

std::vector<DistanceSensorData> Solver::calculateTotalField(const std::vector<DistanceSensorData> &repulsive,
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