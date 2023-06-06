#ifndef _LIDAR_PROCESSOR_H_
#define _LIDAR_PROCESSOR_H_

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include <pcl/common/distances.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "threadsafe_queue.h"

class LidarProcessor
{
public:
    using PointT = pcl::PointXYZ;
    using PointCloudT = pcl::PointCloud<PointT>;
    using PointCloudPtr = pcl::PointCloud<PointT>::Ptr;

    static std::unique_ptr<LidarProcessor> createWithLidarDataQueue(std::shared_ptr<ThreadsafeQueue<PointCloudPtr>> lidarDataQueue);
    ~LidarProcessor() {}
    void start();
    void stop();
    void process();
    std::shared_ptr<ThreadsafeQueue<PointCloudPtr>> getLidarProcessedQueue() const { return mLidarProcessedQueue; }

private:
    LidarProcessor() {}
    bool init(std::shared_ptr<ThreadsafeQueue<PointCloudPtr>> lidarDataQueue);

    std::shared_ptr<ThreadsafeQueue<PointCloudPtr>> mLidarDataQueue;
    std::shared_ptr<ThreadsafeQueue<PointCloudPtr>> mLidarProcessedQueue;
    std::thread mProcessThread;
    std::atomic<bool> mRunning;

    pcl::VoxelGrid<PointT> mVoxelFilter;
    pcl::ConditionalRemoval<PointT> mConditionalRemovalFilter;
    pcl::SACSegmentation<PointT> mSegmentation;
    pcl::ModelCoefficientsPtr mModelCoefficients;
    pcl::PointIndicesPtr mInliers;
    pcl::ExtractIndices<PointT> mExtractor;
};

class RangeCondition : public pcl::ConditionBase<pcl::PointXYZ>
{
public:
    RangeCondition(float range)
        : mRange(range)
    {
    }

    bool evaluate(const pcl::PointXYZ &point) const
    {
        return pcl::euclideanDistance(mOrigin, point) >= mRange;
    }

private:
    float mRange;
    pcl::PointXYZ mOrigin{0.0f, 0.0f, 0.0f};
};

#endif // _LIDAR_PROCESSOR_H_