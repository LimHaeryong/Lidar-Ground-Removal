#ifndef _CARLA_MANAGER_H_
#define _CARLA_MANAGER_H_

#include <atomic>
#include <chrono>
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
        return boost::static_pointer_cast<carla::client::Vehicle>(mVehicleActor);
    }

    boost::shared_ptr<carla::client::Sensor> getLidar() const
    {
        return boost::static_pointer_cast<carla::client::Sensor>(mLidarActor);
    }

    void start();
    void stop();

private:
    CarlaManager()
    {
    }
    bool init(const YAML::Node &config);
    void initLidar(const YAML::Node &config);

    void run();

    std::shared_ptr<carla::client::Client> mClient;
    std::shared_ptr<carla::client::World> mWorld;
    boost::shared_ptr<carla::client::BlueprintLibrary> mBlueprintLibrary;
    boost::shared_ptr<carla::client::Actor> mVehicleActor;
    boost::shared_ptr<carla::client::Actor> mSpectator;
    boost::shared_ptr<carla::client::Actor> mLidarActor;

    std::thread mWorldThread;
    std::chrono::duration<double> mLoopDuration;
    std::atomic<bool> mRunning;
};

#endif // _CARLA_MANAGER_H_