#ifndef ISOLVER_H
#define ISOLVER_H

#include "idataprovider.h"

#include <memory>
#include <string>
#include <vector>

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

class ISolver
{
public:
    void init(std::unique_ptr<IDataProvider> dataProvider);
    std::vector<DistanceSensorData> getSensorData();
    Forces getForces();
    virtual int calculateHeadingAngle() = 0;
protected:
    std::vector<DistanceSensorData> _distanceSensorData;
    Forces _forces;
    std::unique_ptr<IDataProvider> _dataProvider;

    virtual std::vector<DistanceSensorData> calculateRepulsiveField() = 0;
    virtual std::vector<DistanceSensorData> calculateAttractiveField() = 0;
    void calculateForces();
    std::vector<DistanceSensorData> calculateTotalField(const std::vector<DistanceSensorData>& repulsive,
                                                        const std::vector<DistanceSensorData>& attractive);
};
}
#endif // ISOLVER_H
