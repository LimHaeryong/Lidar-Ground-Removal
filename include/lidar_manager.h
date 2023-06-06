#ifndef _LIDAR_MANAGER_H_
#define _LIDAR_MANAGER_H_

#include <atomic>
#include <memory>

#include <boost/smart_ptr.hpp>

#include <carla/client/Sensor.h>
#include <carla/sensor/SensorData.h>
#include <carla/sensor/data/LidarData.h>
#include <carla/sensor/data/LidarMeasurement.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "threadsafe_queue.h"

class LidarManager
{
public:
    using PointT = pcl::PointXYZ;
    using PointCloudT = pcl::PointCloud<PointT>;
    using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;

    static std::unique_ptr<LidarManager> createWithLidar(boost::shared_ptr<carla::client::Sensor> lidar);
    ~LidarManager() {}
    std::shared_ptr<ThreadsafeQueue<PointCloudPtr>> getLidarDataQueue() const { return mLidarDataQueue; }
    void start() { mRunning.store(true); }
    void stop() { mRunning.store(false); }

private:
    LidarManager() {}
    bool init(boost::shared_ptr<carla::client::Sensor> lidar);
    void lidarCallback(boost::shared_ptr<carla::sensor::SensorData> callback);

    PointCloudPtr mBufferCloud;
    int mBufferCount{0};
    boost::shared_ptr<carla::client::Sensor> mLidar;
    std::shared_ptr<ThreadsafeQueue<PointCloudPtr>> mLidarDataQueue;
    std::atomic<bool> mRunning;
};

#endif // _LIDAR_MANAGER_H_