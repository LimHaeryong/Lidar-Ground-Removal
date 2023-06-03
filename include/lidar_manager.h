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

class LidarManager
{
public:
    using PointT = pcl::PointXYZ;
    using PointCloudT = pcl::PointCloud<PointT>;
    using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;

    static std::unique_ptr<LidarManager> createWithLidar(boost::shared_ptr<carla::client::Sensor> lidar);
    ~LidarManager() {}
    std::shared_ptr<std::mutex> getMutex() {return mScanCloudMutex;}
    void getScanCloud(PointCloudT& inputCloud);
    bool hasNewScan() const {return mHasNewScan.load();}

private:
    LidarManager() {}
    bool init(boost::shared_ptr<carla::client::Sensor> lidar);
    void lidarCallback(boost::shared_ptr<carla::sensor::SensorData> callback);

    boost::shared_ptr<carla::client::Sensor> mLidar;
    PointCloudPtr mScanCloud;
    PointCloudPtr mBufferCloud;

    std::shared_ptr<std::mutex> mBufferCloudMutex;
    std::shared_ptr<std::mutex> mScanCloudMutex;
    std::atomic<bool> mHasNewScan{false};
};

#endif // _LIDAR_MANAGER_H_