#ifndef JSONDATAPROVIDER_H
#define JSONDATAPROVIDER_H

#include <idataprovider.h>
#include <string>

class JsonDataProvider : public Solver::IDataProvider
{
public:
    JsonDataProvider(const std::string& path);
    std::vector<Solver::DistanceSensorData> getSample() override;
private:
    std::string _path;
};

#endif // JSONDATAPROVIDER_H
