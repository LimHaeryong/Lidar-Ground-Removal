#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>

#include "carla_manager.h"
#include "lidar_manager.h"
#include "lidar_processor.h"
#include "utils.h"

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;
using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
	SPDLOG_INFO("Start program");
	YAML::Node config;
	try
	{
		if (argc == 2)
		{
			config = YAML::LoadFile(argv[1]);
		}
		else
		{
			config = YAML::LoadFile("./config/config.yaml");
		}
	}
	catch (const std::exception &e)
	{
		SPDLOG_ERROR("failed to load config file: {}", e.what());
		return -1;
	}

	auto carlaManager = CarlaManager::createWithConfig(config);
	if (carlaManager == nullptr)
	{
		SPDLOG_ERROR("failed to create carlaManager");
		return -1;
	}
	auto vehicle = carlaManager->getVehicle();
	auto lidar = carlaManager->getLidar();

	auto viewer = createViewer(config);
	auto lidarManager = LidarManager::createWithLidar(carlaManager->getLidar());
	if (lidarManager == nullptr)
	{
		SPDLOG_ERROR("failed to create lidarManager");
		return -1;
	}
	auto lidarProcessor = LidarProcessor::createWithMutex(lidarManager->getMutex());
	if (lidarProcessor == nullptr)
	{
		SPDLOG_ERROR("failed to create lidarProcessor");
		return -1;
	}
	PointCloudPtr inputCloud = pcl::make_shared<PointCloudT>();
	PointCloudPtr outputCloud = pcl::make_shared<PointCloudT>();
	pcl::visualization::PointCloudColorHandlerCustom<PointT> green(outputCloud, 0, 255, 0);

	SPDLOG_INFO("Start main loop");
	while (!viewer.wasStopped())
	{
		if (!lidarManager->hasNewScan())
		{
			continue;
		}
		lidarManager->getScanCloud(*inputCloud);
		lidarProcessor->setInputCloud(inputCloud);
		lidarProcessor->process(*outputCloud);
		viewer.removeAllPointClouds();
		viewer.addPointCloud(outputCloud, green, "filteredCloud");
		viewer.spinOnce();
	}

	SPDLOG_INFO("End program");
	return 0;
}