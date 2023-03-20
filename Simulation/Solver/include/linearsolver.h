#ifndef LINEARSOLVER_H
#define LINEARSOLVER_H

#include "isolver.h"

namespace Solver
{

class LinearSolver : public ISolver
{
public:
    int calculateHeadingAngle() override;

private:
    std::vector<DistanceSensorData> calculateRepulsiveField() override;
    std::vector<DistanceSensorData> calculateAttractiveField() override;
};

}
#endif // LINEARSOLVER_H
