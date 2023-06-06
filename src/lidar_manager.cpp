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
        double sensor_tick, rotation_frequency;
        for(auto& attribute : mLidar->GetAttributes())
        {
            std::string id = attribute.GetId();
            if(id == "sensor_tick")
            {
                sensor_tick = std::stod(attribute.GetValue());
            }
            if(id == "rotation_frequency")
            {
                rotation_frequency = std::stod(attribute.GetValue());
            }
        }
        mBufferMaxCount = static_cast<uint16_t>(1.0 / sensor_tick / rotation_frequency);
        mLidar->Listen([this](auto callback)
                       {
                           if (mRunning.load() == true)
                           {
                               lidarCallback(callback);
                           }
                       });
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
    auto scan = boost::static_pointer_cast<carla::sensor::data::LidarMeasurement>(callback);
    if(mBufferCount == 0)
    {
        mBufferCloud = pcl::make_shared<PointCloudT>();
        mBufferCloud->reserve(scan->size() * mBufferMaxCount);
    }
    ++mBufferCount;
    for (auto s : *scan)
    {
        mBufferCloud->points.emplace_back(s.point.x, -s.point.y, s.point.z);
    }
    if(mBufferCount == mBufferMaxCount)
    {
        mLidarDataQueue->push(std::move(mBufferCloud));
        mBufferCloud.reset();
        mBufferCount = 0;
    }
}
