#include <exception>

#include <spdlog/spdlog.h>

#include "lidar_processor.h"

std::unique_ptr<LidarProcessor> LidarProcessor::createWithMutex(std::shared_ptr<std::mutex> scanCloudMutex)
{
    auto lidarProcessor = std::unique_ptr<LidarProcessor>(new LidarProcessor());
    if (!lidarProcessor->init(scanCloudMutex))
    {
        return nullptr;
    }
    return std::move(lidarProcessor);
}

bool LidarProcessor::init(std::shared_ptr<std::mutex> scanCloudMutex)
{
    try
    {
        mScanCloudMutex = scanCloudMutex;
        mFilteredCloud = pcl::make_shared<PointCloudT>();
        mVoxelFilter.setLeafSize(0.1f, 0.1f, 0.1f);
        return true;
    }
    catch (const std::exception &e)
    {
        SPDLOG_ERROR("failed to initialize lidarProcessor. {}", e.what());
        return false;
    }
}

void LidarProcessor::setInputCloud(PointCloudPtr inputCloud)
{
    std::unique_lock<std::mutex> lockInput(*mScanCloudMutex);
    mVoxelFilter.setInputCloud(inputCloud);
    mVoxelFilter.filter(*mFilteredCloud);
}

void LidarProcessor::process(PointCloudT &outputCloud)
{
    outputCloud.swap(*mFilteredCloud);
}