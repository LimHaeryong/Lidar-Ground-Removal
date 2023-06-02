#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>

#include <carla/geom/Transform.h>
#include <carla/sensor/data/LidarData.h>
#include <carla/sensor/data/LidarMeasurement.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include "carla_manager.h"
#include "utils.h"

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;
using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
	SPDLOG_INFO("Start program");
	YAML::Node config;
	try
	{
		if(argc == 2)
		{
			config = YAML::LoadFile(argv[1]);
		}
		else
		{
			config = YAML::LoadFile("./config/config.yaml");
		}
	}
	catch(const std::exception& e)
	{
		SPDLOG_ERROR("failed to load config file: {}", e.what());
		return -1;
	}
	
	auto carlaManager = CarlaManager::createWithConfig(config);
	if(carlaManager == nullptr)
	{
		SPDLOG_ERROR("failed to create carlaManager");
		return -1;
	}

	auto vehicle = carlaManager->getVehicle();
	auto lidar = carlaManager->getLidar();

	PointCloudPtr scanCloud = pcl::make_shared<PointCloudT>();
	PointCloudPtr bufferCloud = pcl::make_shared<PointCloudT>();
	PointCloudPtr filteredCloud = pcl::make_shared<PointCloudT>();

	pcl::VoxelGrid<PointT> voxelFilter;
	voxelFilter.setLeafSize(0.1f, 0.1f, 0.1f);

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

	auto viewer = createViewer(config);
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
       	viewer.removeAllPointClouds();
		viewer.addPointCloud(filteredCloud, green, "filteredCloud");
		newScan = true;
	}
	lidar->Destroy();
	vehicle->Destroy();
	SPDLOG_INFO("End program");
	return 0;
}