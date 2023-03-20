#ifndef LAPLACESOLVER_H
#define LAPLACESOLVER_H

#include "isolver.h"

namespace Solver
{

class LaplaceSolver : public ISolver
{
public:
    int calculateHeadingAngle() override;

private:
    std::vector<DistanceSensorData> calculateRepulsiveField() override;
    std::vector<DistanceSensorData> calculateAttractiveField() override;
};

}

#endif // LAPLACESOLVER_H
