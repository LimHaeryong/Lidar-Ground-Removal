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
		SPDLOG_ERROR("Failed to load config file: {}", e.what());
		return -1;
	}

	auto carlaManager = CarlaManager::createWithConfig(config);
	if (carlaManager == nullptr)
	{
		SPDLOG_ERROR("Failed to create carlaManager");
		return -1;
	}

	auto lidarManager = LidarManager::createWithLidar(carlaManager->getLidar());
	if (lidarManager == nullptr)
	{
		SPDLOG_ERROR("Failed to create lidarManager");
		return -1;
	}

	auto lidarProcessor = LidarProcessor::createWithLidarDataQueue(lidarManager->getLidarDataQueue());
	if (lidarProcessor == nullptr)
	{
		SPDLOG_ERROR("Failed to create lidarProcessor");
		return -1;
	}

	auto outputCloud = pcl::make_shared<PointCloudT>();

	auto viewer = createViewer(config);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> green(outputCloud, 0, 255, 0);

	SPDLOG_INFO("Start main loop");
	while (!viewer.wasStopped())
	{
		lidarProcessor->process(*outputCloud);
		viewer.removeAllPointClouds();
		viewer.addPointCloud(outputCloud, green, "outputCloud");
		viewer.spinOnce();
	}

	SPDLOG_INFO("End program");
	return 0;
}