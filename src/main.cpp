#include <carla/client/Client.h>
#include <carla/client/BlueprintLibrary.h>

#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

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

	auto lidarLocation = carla::geom::Location(LIDAR_LOCATION[0], LIDAR_LOCATION[1], LIDAR_LOCATION[2]);
	auto lidarRotation = carla::geom::Rotation(LIDAR_ROTATION[0], LIDAR_ROTATION[1], LIDAR_ROTATION[2]);
	auto lidarTransform = carla::geom::Transform(lidarLocation, lidarRotation);
	auto lidarActor = world.SpawnActor(lidarModel, lidarTransform, vehicleActor.get());

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

		SPDLOG_INFO("loop");
		std::this_thread::sleep_for(1s);
	}

	vehicle->Destroy();
	SPDLOG_INFO("End program");
	return 0;
}