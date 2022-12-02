#ifndef IDATAPROVIDER_H
#define IDATAPROVIDER_H

#include <vector>
namespace Solver
{

struct DistanceSensorData
{
    int angle;
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
