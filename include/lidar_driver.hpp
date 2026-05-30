#pragma once

#include <lidar_driver_impl.hpp>
#include <utility/sync_queue.hpp>

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>

namespace robosense
{
namespace lidar
{

std::string getDriverVersion();

template <typename T_PointCloud>
class LidarDriver
{
public:
  LidarDriver()
    : dropped_cloud_count_(0)
    , initialized_(false)
    , started_(false)
    , driver_ptr_(std::make_shared<LidarDriverImpl<T_PointCloud>>())
  {
    useDefaultPointCloudQueue();
    useDefaultExceptionHandler();
  }

  ~LidarDriver()
  {
    stop();
  }

  LidarDriver(const LidarDriver&) = delete;
  LidarDriver& operator=(const LidarDriver&) = delete;

  inline bool getPointCloud(std::shared_ptr<T_PointCloud>& cloud, unsigned int usec = 1000000)
  {
    cloud = ready_cloud_queue_.popWait(usec);
    return static_cast<bool>(cloud);
  }

  inline void recyclePointCloud(std::shared_ptr<T_PointCloud>& cloud)
  {
    if (!cloud)
    {
      return;
    }

    pushFreePointCloud(cloud);
    cloud.reset();
  }

  inline bool isInitialized() const
  {
    return initialized_.load(std::memory_order_acquire);
  }

  inline bool isStarted() const
  {
    return started_.load(std::memory_order_acquire);
  }

  inline uint64_t droppedPointCloudCount() const
  {
    return dropped_cloud_count_.load(std::memory_order_acquire);
  }

  inline size_t readyPointCloudSize() const
  {
    return ready_cloud_queue_.size();
  }

  inline void setExceptionCallback(const std::function<void(const Error&)>& cb_excep)
  {
    cb_exception_ = cb_excep ? cb_excep : std::bind(&LidarDriver<T_PointCloud>::defaultExceptionHandler, this,
                                                    std::placeholders::_1);
    driver_ptr_->regExceptionCallback(cb_exception_);
  }

  inline bool init(const RSDriverParam& param)
  {
    if (initialized_.load(std::memory_order_acquire))
    {
      return true;
    }

    bool ok = driver_ptr_->init(param);
    initialized_.store(ok, std::memory_order_release);
    return ok;
  }

  inline bool start()
  {
    if (started_.load(std::memory_order_acquire))
    {
      return true;
    }

    if (!initialized_.load(std::memory_order_acquire))
    {
      reportException(Error(ERRCODE_STARTBEFOREINIT));
      return false;
    }

    bool ok = driver_ptr_->start();
    started_.store(ok, std::memory_order_release);
    return ok;
  }

  inline void stop()
  {
    if (!started_.load(std::memory_order_acquire))
    {
      return;
    }

    driver_ptr_->stop();
    started_.store(false, std::memory_order_release);
  }

private:
  inline void useDefaultPointCloudQueue()
  {
    driver_ptr_->regPointCloudCallback(
        std::bind(&LidarDriver<T_PointCloud>::getReusablePointCloud, this),
        std::bind(&LidarDriver<T_PointCloud>::putReadyPointCloud, this, std::placeholders::_1));
  }

  inline void useDefaultExceptionHandler()
  {
    cb_exception_ = std::bind(&LidarDriver<T_PointCloud>::defaultExceptionHandler, this, std::placeholders::_1);
    driver_ptr_->regExceptionCallback(cb_exception_);
  }

  inline void reportException(const Error& error)
  {
    if (cb_exception_)
    {
      cb_exception_(error);
    }
  }

  inline void defaultExceptionHandler(const Error& error)
  {
    RS_WARNING << error.toString() << RS_REND;
  }

  inline std::shared_ptr<T_PointCloud> getReusablePointCloud()
  {
    std::shared_ptr<T_PointCloud> cloud = free_cloud_queue_.pop();
    return cloud ? cloud : std::make_shared<T_PointCloud>();
  }

  inline void putReadyPointCloud(std::shared_ptr<T_PointCloud> cloud)
  {
    pushReadyPointCloud(cloud);
  }

  inline void pushReadyPointCloud(const std::shared_ptr<T_PointCloud>& cloud)
  {
    if (!cloud)
    {
      return;
    }

    if (ready_cloud_queue_.push(cloud) == static_cast<size_t>(-1))
    {
      dropped_cloud_count_.fetch_add(1, std::memory_order_acq_rel);
      reportException(Error(ERRCODE_CLOUDOVERFLOW));
      pushFreePointCloud(cloud);
    }
  }

  inline void pushFreePointCloud(const std::shared_ptr<T_PointCloud>& cloud)
  {
    if (!cloud)
    {
      return;
    }

    if (free_cloud_queue_.push(cloud) == static_cast<size_t>(-1))
    {
      dropped_cloud_count_.fetch_add(1, std::memory_order_acq_rel);
      reportException(Error(ERRCODE_CLOUDOVERFLOW));
    }
  }

  SyncQueue<std::shared_ptr<T_PointCloud>> free_cloud_queue_;
  SyncQueue<std::shared_ptr<T_PointCloud>> ready_cloud_queue_;
  std::atomic<uint64_t> dropped_cloud_count_;
  std::atomic<bool> initialized_;
  std::atomic<bool> started_;
  std::function<void(const Error&)> cb_exception_;
  std::shared_ptr<LidarDriverImpl<T_PointCloud>> driver_ptr_;
};

}  // namespace lidar
}  // namespace robosense
