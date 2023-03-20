#include "jsondataprovider.h"

#include <QFile>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>

namespace
{
const char* kDataKey{"Data"};
const char* kAngleKey{"angle"};
const char* kDistanceKey{"distance"};
}

JsonDataProvider::JsonDataProvider(const std::string& path) : _path{path}
{ }

std::vector<Solver::DistanceSensorData> JsonDataProvider::getSample()
{
    std::vector<Solver::DistanceSensorData> sample;
    QFile file(_path.c_str());

    if (!file.open(QIODevice::ReadOnly))
    {
        qWarning() << "failed to open file:" << file.errorString();
        return {};
    }

    QByteArray rawData = file.readAll();

    QJsonObject jsonObj = QJsonDocument::fromJson(rawData).object();
    if (jsonObj.contains(kDataKey) && jsonObj[kDataKey].isArray())
    {
        qInfo() << "parsing json...";
        QJsonArray array = jsonObj[kDataKey].toArray();

        for (int idx = 0; idx < array.size(); ++idx)
        {
            sample.push_back({/*Solver::DegreesToRadians*/(array[idx].toObject()[kAngleKey].toDouble()),
                              array[idx].toObject()[kDistanceKey].toDouble()});
        }
    }

    file.close();
    return sample;
}
