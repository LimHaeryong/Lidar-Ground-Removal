#ifndef _LIDAR_PROCESSOR_H_
#define _LIDAR_PROCESSOR_H_

#include <memory>
#include <mutex>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h> 

class LidarProcessor
{
public:
    using PointT = pcl::PointXYZ;
    using PointCloudT = pcl::PointCloud<PointT>;
    using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;

    static std::unique_ptr<LidarProcessor> createWithMutex(std::shared_ptr<std::mutex> scanCloudMutex);
    void setInputCloud(PointCloudPtr inputCloud);
    void process(PointCloudT& outputCloud);

private:
    LidarProcessor() {}
    bool init(std::shared_ptr<std::mutex> scanCloudMutex);

    std::shared_ptr<std::mutex> mScanCloudMutex;
    PointCloudPtr mFilteredCloud;    
    pcl::VoxelGrid<PointT> mVoxelFilter;
};

#endif // _LIDAR_PROCESSOR_H_