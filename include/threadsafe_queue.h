#ifndef _THREADSAFE_QUEUE_
#define _THREADSAFE_QUEUE_

#include <condition_variable>
#include <mutex>
#include <queue>

#include <spdlog/spdlog.h>

template <typename T>
class ThreadsafeQueue
{
public:
    ThreadsafeQueue() {}
    ~ThreadsafeQueue() {}

    void push(const T& data)
    {
        std::unique_lock<std::mutex> lock(mMutex);
        mData.push(data);
        mConditionVariable.notify_one();
    }

    void push(T&& data)
    {
        std::unique_lock<std::mutex> lock(mMutex);
        mData.push(std::move(data));
        mConditionVariable.notify_one();
    }

    T pop()
    {
        std::unique_lock<std::mutex> lock(mMutex);
        mConditionVariable.wait(lock, [this]
        {
            return !mData.empty();
        });
        T value = std::move(mData.front());
        mData.pop();
        return value;
    }
    
private:
    std::queue<T> mData;
    std::mutex mMutex;
    std::condition_variable mConditionVariable;
};

#endif // _THREADSAFE_QUEUE_
