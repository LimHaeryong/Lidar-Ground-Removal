#ifndef _CARLA_MANAGER_H_
#define _CARLA_MANAGER_H_

#include <memory>

#include <boost/smart_ptr.hpp>
#include <yaml-cpp/yaml.h>

#include <carla/client/Actor.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Sensor.h>
#include <carla/client/Vehicle.h>

class CarlaManager
{
public:
    static std::unique_ptr<CarlaManager> createWithConfig(const YAML::Node &config);
    ~CarlaManager();

    boost::shared_ptr<carla::client::Vehicle> getVehicle() const
    {
        return mVehicle;
    }

    boost::shared_ptr<carla::client::Sensor> getLidar() const
    {
        return mLidar;
    }

private:
    CarlaManager()
    {
    }
    bool init(const YAML::Node &config);
    void initLidar(const YAML::Node &config);

    std::shared_ptr<carla::client::Client> mClient;
    std::shared_ptr<carla::client::World> mWorld;
    boost::shared_ptr<carla::client::BlueprintLibrary> mBlueprintLibrary;
    boost::shared_ptr<carla::client::Vehicle> mVehicle;
    boost::shared_ptr<carla::client::Actor> mSpectator;
    boost::shared_ptr<carla::client::Sensor> mLidar;
};

#endif // _CARLA_MANAGER_H_