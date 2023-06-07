#include <exception>
#include <mutex>
#include <iostream>

#include <spdlog/spdlog.h>

#include "lidar_manager.h"

std::unique_ptr<LidarManager> LidarManager::createWithLidar(boost::shared_ptr<carla::client::Sensor> lidar)
{
    auto lidarManager = std::unique_ptr<LidarManager>(new LidarManager());
    if (!lidarManager->init(lidar))
    {
        return nullptr;
    }
    return std::move(lidarManager);
}

bool LidarManager::init(boost::shared_ptr<carla::client::Sensor> lidar)
{
    try
    {
        mLidarDataQueue = std::make_shared<ThreadsafeQueue<PointCloudPtr>>();
        mLidar = lidar;
        mRunning.store(false);
        mLidar->Listen([this](auto callback)
                       {
                           if (mRunning.load() == true)
                           {
                               lidarCallback(callback);
                           } });
        return true;
    }
    catch (const std::exception &e)
    {
        SPDLOG_ERROR("Lidar init failed. {}", e.what());
        return false;
    }
}

void LidarManager::lidarCallback(boost::shared_ptr<carla::sensor::SensorData> callback)
{
    std::unique_lock<std::mutex> lock(mLidarCallbackMutex, std::try_to_lock);
    if(!lock.owns_lock())
    {
        return;
    }
    auto scan = boost::static_pointer_cast<carla::sensor::data::LidarMeasurement>(callback);

    mBufferCloud = pcl::make_shared<PointCloudT>();
    mBufferCloud->reserve(scan->size());
    for (auto s : *scan)
    {
        mBufferCloud->points.emplace_back(s.point.x, -s.point.y, s.point.z);
    }
    mLidarDataQueue->push(std::move(mBufferCloud));
}
