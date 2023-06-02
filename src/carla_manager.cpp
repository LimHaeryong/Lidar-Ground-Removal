#include <chrono>
#include <cstdint>
#include <exception>
#include <vector>

#include <spdlog/spdlog.h>

#include <carla/client/BlueprintLibrary.h>
#include <carla/geom/Transform.h>

#include "carla_manager.h"

std::unique_ptr<CarlaManager> CarlaManager::createWithConfig(const YAML::Node& config)
{
    auto carlaManager = std::unique_ptr<CarlaManager>(new CarlaManager());
    if(!carlaManager->init(config))
    {
        return nullptr;
    }
    return std::move(carlaManager);
}

CarlaManager::~CarlaManager()
{
    mVehicleActor->Destroy();
    mLidarActor->Destroy();
}

bool CarlaManager::init(const YAML::Node& config)
{
    using namespace std::chrono_literals;
    try
    {
        const auto HOST = config["CLIENT"]["HOST"].as<std::string>();
        const auto PORT = config["CLIENT"]["PORT"].as<uint16_t>();
        const auto FIXED_DELTA_SECONDS = config["WORLD"]["FIXED_DELTA_SECONDS"].as<double>();
        const auto MAP = config["WORLD"]["MAP"].as<std::string>();
        const auto VEHICLE_ID = config["VEHICLE_ID"].as<std::string>();
        const auto LIDAR = config["SENSOR"]["LIDAR"];
        const auto LIDAR_LOCATION = LIDAR["LOCATION"].as<std::vector<float>>();
        const auto LIDAR_ROTATION = LIDAR["ROTATION"].as<std::vector<float>>();
        const auto VOXEL_RESOLUTION = config["VOXEL_FILTER"]["RESOLUTION"].as<float>();

        mClient = std::make_shared<carla::client::Client>(HOST, PORT);
        mClient->SetTimeout(2s);
        
        SPDLOG_INFO("Load world {}", MAP);
        mClient->LoadWorld(config["WORLD"]["MAP"].as<std::string>());
        
        mWorld = std::make_shared<carla::client::World>(mClient->GetWorld());
        auto worldSetting = mWorld->GetSettings();
        worldSetting.fixed_delta_seconds = FIXED_DELTA_SECONDS;
        mWorld->ApplySettings(worldSetting, 0s);
        
        mBlueprintLibrary = mWorld->GetBlueprintLibrary();
        auto vehicleLibrary = mBlueprintLibrary->Filter("vehicle");
        auto vehicleModel = *(vehicleLibrary->Find(VEHICLE_ID));

        auto map = mWorld->GetMap();
        auto spawnPoint = map->GetRecommendedSpawnPoints().front();
        auto mVehicleActor = mWorld->SpawnActor(vehicleModel, spawnPoint);

        mSpectator = mWorld->GetSpectator();
        auto specTransform = spawnPoint;
        specTransform.location -= 10.0f * specTransform.GetForwardVector();
        specTransform.location.z += 4.0f;
        specTransform.rotation.pitch = -30.0f;
        mSpectator->SetTransform(specTransform);

        if(config["SENSOR"]["LIDAR"].IsDefined())
        {
            initLidar(config);
        }

        return true;
    }
    catch(const std::exception& e)
    {
        SPDLOG_ERROR("failed to initialize CarlaManager: {}", e.what());
        return false;
    }
}

void CarlaManager::initLidar(const YAML::Node& config)
{
    const auto LIDAR = config["SENSOR"]["LIDAR"];
    const auto LIDAR_LOCATION = LIDAR["LOCATION"].as<std::vector<float>>();
    const auto LIDAR_ROTATION = LIDAR["ROTATION"].as<std::vector<float>>();

    auto lidarModel = *(mBlueprintLibrary->Find(LIDAR["MODEL"].as<std::string>()));
    lidarModel.SetAttribute("upper_fov", LIDAR["UPPER_FOV"].as<std::string>());
    lidarModel.SetAttribute("lower_fov", LIDAR["LOWER_FOV"].as<std::string>());
    lidarModel.SetAttribute("channels", LIDAR["CHANNELS"].as<std::string>());
    lidarModel.SetAttribute("range", LIDAR["RANGE"].as<std::string>());
    lidarModel.SetAttribute("rotation_frequency", LIDAR["ROTATION_FREQUENCY"].as<std::string>());
    lidarModel.SetAttribute("points_per_second", LIDAR["POINTS_PER_SECOND"].as<std::string>());
    lidarModel.SetAttribute("sensor_tick", LIDAR["SENSOR_TICK"].as<std::string>());
    lidarModel.SetAttribute("noise_stddev", LIDAR["NOISE_STDDEV"].as<std::string>());

    auto lidarTransform = carla::geom::Transform(carla::geom::Location(LIDAR_LOCATION[0], LIDAR_LOCATION[1], LIDAR_LOCATION[2]), 
    											 carla::geom::Rotation(LIDAR_ROTATION[0], LIDAR_ROTATION[1], LIDAR_ROTATION[2]));
    mLidarActor = mWorld->SpawnActor(lidarModel, lidarTransform, mVehicleActor.get());
}