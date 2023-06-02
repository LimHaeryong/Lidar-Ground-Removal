#include <carla/client/Client.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Sensor.h>
#include <carla/geom/Transform.h>
#include <carla/sensor/data/LidarData.h>
#include <carla/sensor/data/LidarMeasurement.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>

#include <boost/smart_ptr.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
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
	const auto FIXED_DELTA_SECONDS = config["WORLD"]["FIXED_DELTA_SECONDS"].as<double>();
	const auto MAP = config["WORLD"]["MAP"].as<std::string>();
	const auto VEHICLE_ID = config["VEHICLE_ID"].as<std::string>();
	const auto LIDAR = config["SENSOR"]["LIDAR"];
	const auto LIDAR_LOCATION = LIDAR["LOCATION"].as<std::vector<float>>();
	const auto LIDAR_ROTATION = LIDAR["ROTATION"].as<std::vector<float>>();
	const auto VOXEL_RESOLUTION = config["VOXEL_FILTER"]["RESOLUTION"].as<float>();

	SPDLOG_INFO("Initialize client");
	auto client = carla::client::Client(HOST, PORT);
	client.SetTimeout(2s);
	SPDLOG_INFO("Load world {}", MAP);
	client.LoadWorld(MAP);

	auto world = client.GetWorld();
	auto worldSetting = world.GetSettings();
	worldSetting.fixed_delta_seconds = FIXED_DELTA_SECONDS;
	world.ApplySettings(worldSetting, 0s);

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
	lidarModel.SetAttribute("sensor_tick", LIDAR["SENSOR_TICK"].as<std::string>());
	lidarModel.SetAttribute("noise_stddev", LIDAR["NOISE_STDDEV"].as<std::string>());

	auto lidarTransform = carla::geom::Transform(carla::geom::Location(LIDAR_LOCATION[0], LIDAR_LOCATION[1], LIDAR_LOCATION[2]), 
												 carla::geom::Rotation(LIDAR_ROTATION[0], LIDAR_ROTATION[1], LIDAR_ROTATION[2]));
	auto lidarActor = world.SpawnActor(lidarModel, lidarTransform, vehicleActor.get());
	auto lidar = boost::static_pointer_cast<carla::client::Sensor>(lidarActor);

	PointCloudPtr scanCloud = pcl::make_shared<PointCloudT>();
	PointCloudPtr bufferCloud = pcl::make_shared<PointCloudT>();
	PointCloudPtr filteredCloud = pcl::make_shared<PointCloudT>();

	pcl::VoxelGrid<PointT> voxelFilter;
	voxelFilter.setLeafSize(VOXEL_RESOLUTION, VOXEL_RESOLUTION, VOXEL_RESOLUTION);

	std::mutex scanCloudMutex, bufferCloudMutex, viewerMutex;
	std::atomic<bool> newScan(false);
	std::chrono::time_point<std::chrono::system_clock> lastScanTime;
	lidar->Listen([&scanCloudMutex, &bufferCloudMutex, &newScan, &lastScanTime, &scanCloud, &bufferCloud](auto callback)
	{
		if(newScan)
		{
			std::unique_lock<std::mutex> lockBuffer(bufferCloudMutex, std::try_to_lock);
			if(!lockBuffer.owns_lock())
			{
				return;
			}
			auto scan = boost::static_pointer_cast<carla::sensor::data::LidarMeasurement>(callback);
			bufferCloud->points.clear();
			bufferCloud->points.reserve(scan->size());
			auto originLocation = carla::geom::Location(0.0f, 0.0f, 0.0f);
			double a = (*scan).GetTimestamp();
			SPDLOG_INFO("lidar timestamp : {}", a);
			for(auto s : *scan)
			{
				if(originLocation.Distance(s.point) > 5.0f)
				{
					bufferCloud->points.emplace_back(s.point.x, -s.point.y, s.point.z);
				}
			}
			{
				std::unique_lock<std::mutex> lockScan(scanCloudMutex);
				bufferCloud.swap(scanCloud);
				lastScanTime = std::chrono::system_clock::now();
				
			}
			newScan = false;
		}
	});

	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	viewer.addCoordinateSystem (1.0);
	viewer.setCameraPosition(-100, 0, 50, 0, 0, 0, 0, 0, 1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(filteredCloud, 0, 255, 0);

	bool exitLoop = false;
	SPDLOG_INFO("Start main loop");
	while(!viewer.wasStopped())
	{
		viewer.spinOnce();
		if(newScan)
		{
			continue;
		}

		{
			std::lock_guard<std::mutex> lockScan(scanCloudMutex);
			voxelFilter.setInputCloud(scanCloud);
			voxelFilter.filter(*filteredCloud);
			
		}

		viewer.removeAllShapes();
       	viewer.removeAllPointClouds();
		viewer.addPointCloud(filteredCloud, green, "filteredCloud");
	
		newScan = true;
		SPDLOG_INFO("show filtered cloud. cloud size : {}", filteredCloud->points.size());
	}

	lidar->Destroy();
	vehicle->Destroy();
	SPDLOG_INFO("End program");
	return 0;
}