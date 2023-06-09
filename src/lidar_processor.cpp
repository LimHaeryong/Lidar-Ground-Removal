#include <exception>

#include <spdlog/spdlog.h>

#include "lidar_processor.h"

std::unique_ptr<LidarProcessor> LidarProcessor::createWithLidarDataQueue(std::shared_ptr<ThreadsafeQueue<PointCloudPtr>> lidarDataQueue)
{
    auto lidarProcessor = std::unique_ptr<LidarProcessor>(new LidarProcessor());
    if (!lidarProcessor->init(lidarDataQueue))
    {
        return nullptr;
    }
    return std::move(lidarProcessor);
}

bool LidarProcessor::init(std::shared_ptr<ThreadsafeQueue<PointCloudPtr>> lidarDataQueue)
{
    try
    {
        mLidarDataQueue = lidarDataQueue;
        mLidarProcessedQueue = std::make_shared<ThreadsafeQueue<PointCloudPtr>>();
        mRunning.store(false);

        mVoxelFilter.setLeafSize(1.0f, 1.0f, 1.0f);
        auto rangeCondition = pcl::make_shared<RangeCondition>(3.0f);
        mConditionalRemovalFilter.setCondition(rangeCondition);
        mModelCoefficients = pcl::make_shared<pcl::ModelCoefficients>();
        mInliers = pcl::make_shared<pcl::PointIndices>();
        mSegmentation.setOptimizeCoefficients(true);
        mSegmentation.setModelType(pcl::SACMODEL_PLANE);
        mSegmentation.setMaxIterations(500);
        mSegmentation.setDistanceThreshold(0.3);
        mExtractor.setNegative(true);

        return true;
    }
    catch (const std::exception &e)
    {
        SPDLOG_ERROR("failed to initialize lidarProcessor. {}", e.what());
        return false;
    }
}

void LidarProcessor::start()
{
    mRunning.store(true);
    mProcessThread = std::thread(&LidarProcessor::process, this);
}

void LidarProcessor::stop()
{
    mRunning.store(false);
    if (mProcessThread.joinable())
    {
        mProcessThread.join();
    }
}

void LidarProcessor::process()
{
    while (mRunning.load() == true)
    {
        auto cloud = mLidarDataQueue->pop();
        // mVoxelFilter.setInputCloud(cloud);
        // mVoxelFilter.filter(*cloud);
        mConditionalRemovalFilter.setInputCloud(cloud);
        mConditionalRemovalFilter.filter(*cloud);
        for (int i = 0; i < 2; ++i)
        {
            mSegmentation.setInputCloud(cloud);
            mSegmentation.segment(*mInliers, *mModelCoefficients);
            mExtractor.setInputCloud(cloud);
            mExtractor.setIndices(mInliers);
            mExtractor.filter(*cloud);
        }
        mLidarProcessedQueue->push(cloud);
    }
}