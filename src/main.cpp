#include <carla/client/Client.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Sensor.h>
#include <carla/geom/Transform.h>
#include <carla/sensor/data/LidarData.h>
#include <carla/sensor/data/LidarMeasurement.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;
using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;

using namespace std::chrono_literals;

int main()
{
	SPDLOG_INFO("Start program");

	YAML::Node config = YAML::LoadFile("./config/config.yaml");
	if(!config.IsDefined())
	{
		SPDLOG_INFO("Config load failed.");
		return 1;
	}

	const auto HOST = config["CLIENT"]["HOST"].as<std::string>();
	const auto PORT = config["CLIENT"]["PORT"].as<uint16_t>();
	const auto MAP = config["MAP"].as<std::string>();
	const auto VEHICLE_ID = config["VEHICLE_ID"].as<std::string>();
	const auto LIDAR = config["SENSOR"]["LIDAR"];
	const auto LIDAR_LOCATION = LIDAR["LOCATION"].as<std::vector<float>>();
	const auto LIDAR_ROTATION = LIDAR["ROTATION"].as<std::vector<float>>();

	SPDLOG_INFO("Initialize client");
	auto client = carla::client::Client(HOST, PORT);
	client.SetTimeout(2s);
	SPDLOG_INFO("Load world {}", MAP);
	client.LoadWorld(MAP);

	auto world = client.GetWorld();
	auto bluePrintLibrary = world.GetBlueprintLibrary();
	auto vehicleLibrary = bluePrintLibrary->Filter("vehicle");
	auto vehicleModel = *(vehicleLibrary->Find(VEHICLE_ID));
	
	auto map = world.GetMap();
	auto spawnPoint = map->GetRecommendedSpawnPoints().front();
	auto vehicleActor = world.SpawnActor(vehicleModel, spawnPoint);
	auto vehicle = boost::static_pointer_cast<carla::client::Vehicle>(vehicleActor);

	auto spectator = world.GetSpectator();
	auto specTransform = spawnPoint;
	specTransform.location += 32.0f * specTransform.GetForwardVector();
    specTransform.location.z += 2.0f;
    specTransform.rotation.yaw += 180.0f;
    specTransform.rotation.pitch = -15.0f;
    spectator->SetTransform(specTransform);

	auto lidarModel = *(bluePrintLibrary->Find(LIDAR["MODEL"].as<std::string>()));
	lidarModel.SetAttribute("upper_fov", LIDAR["UPPER_FOV"].as<std::string>());
	lidarModel.SetAttribute("lower_fov", LIDAR["LOWER_FOV"].as<std::string>());
	lidarModel.SetAttribute("channels", LIDAR["CHANNELS"].as<std::string>());
	lidarModel.SetAttribute("range", LIDAR["RANGE"].as<std::string>());
	lidarModel.SetAttribute("rotation_frequency", LIDAR["ROTATION_FREQUENCY"].as<std::string>());
	lidarModel.SetAttribute("points_per_second", LIDAR["POINTS_PER_SECOND"].as<std::string>());

	auto lidarTransform = carla::geom::Transform(carla::geom::Location(LIDAR_LOCATION[0], LIDAR_LOCATION[1], LIDAR_LOCATION[2]), 
												 carla::geom::Rotation(LIDAR_ROTATION[0], LIDAR_ROTATION[1], LIDAR_ROTATION[2]));
	auto lidarActor = world.SpawnActor(lidarModel, lidarTransform, vehicleActor.get());
	auto lidar = boost::static_pointer_cast<carla::client::Sensor>(lidarActor);

	PointCloudPtr scanCloud = pcl::make_shared<PointCloudT>();
	PointCloudPtr bufferCloud = pcl::make_shared<PointCloudT>();
	PointCloudPtr fileredCloud = pcl::make_shared<PointCloudT>();

	bool newScan = false;
	std::chrono::time_point<std::chrono::system_clock> lastScanTime;
	lidar->Listen([&newScan, &lastScanTime, &scanCloud, &bufferCloud](auto callback)
	{
		if(newScan)
		{
			bufferCloud->points.clear();
			auto scan = boost::static_pointer_cast<carla::sensor::data::LidarMeasurement>(callback);
			auto originLocation = carla::geom::Location(0.0f, 0.0f, 0.0f);
			for(auto s : *scan)
			{
				if(originLocation.Distance(s.point) > 5.0f)
				{
					bufferCloud->points.emplace_back(s.point.x, s.point.y, s.point.z);
				}
			}
			bufferCloud->swap(*scanCloud);
			lastScanTime = std::chrono::system_clock::now();
			newScan = false;
		}
	});

	bool exitLoop = false;
	SPDLOG_INFO("Start main loop");
	while(!exitLoop)
	{
		if(std::cin.peek() != EOF)
		{
			char key;
			std::cin >> key;
			if(key == 27)
			{
				exitLoop = true;
			}
		}

		

	}

	vehicle->Destroy();
	SPDLOG_INFO("End program");
	return 0;
}