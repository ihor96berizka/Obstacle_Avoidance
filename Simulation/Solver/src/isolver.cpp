#include "isolver.h"

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

}
