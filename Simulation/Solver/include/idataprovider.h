#ifndef IDATAPROVIDER_H
#define IDATAPROVIDER_H

#include <vector>
#include <cmath>

namespace Solver
{

constexpr double DegreesToRadians(double degrees) {
    return degrees * (M_PI / 180);
}

constexpr double RadiansToDegrees(double radians) {
    return radians * (180.0 / M_PI);
}

struct DistanceSensorData
{
    double angle;  // in radians.
    double distance;
};

class IDataProvider
{
public:
/*
*   @brief Fetch data from some source.
*   This should be blocking call.
*/
    virtual std::vector<DistanceSensorData> getSample() = 0;
};

}//  namespace Solver
#endif // IDATAPROVIDER_H
