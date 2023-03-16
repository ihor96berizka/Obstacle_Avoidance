#ifndef SOLVER_H
#define SOLVER_H

#include "idataprovider.h"
#include "isolver.h"

namespace Solver
{

/*
* Usage: Create instance of Solver.
* Flow:
*   init(provider)
*   calculateHeadingAngle()
* calculateHeadingAngle() will wait for new chuck of data and then perform calculations.
* It should be called in a working loop in user code.
*/
class GussianSolver : public ISolver
{
public:
    void init(std::unique_ptr<IDataProvider> dataProvider);
    std::vector<DistanceSensorData> getSensorData();
    Forces getForces();
    int calculateHeadingAngle() override;

private:
    std::vector<Obstacle> enlargeObstacles(const double w_robot);
    std::vector<Obstacle> findObstacles();

    void calculateObstaclesAverages(std::vector<Obstacle> &obstacles);
    std::vector<std::vector<DistanceSensorData>> getRepulsiceComponents();
    std::vector<DistanceSensorData> calculateRepulsiveField() override;
    std::vector<DistanceSensorData> calculateAttractiveField() override;

private:

};

} //namespace Solver
#endif // SOLVER_H
