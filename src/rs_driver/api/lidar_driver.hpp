/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once

#include <rs_driver/driver/lidar_driver_impl.hpp>
#include <rs_driver/msg/packet.hpp>
#include <rs_driver/utility/sync_queue.hpp>

#include <atomic>
#include <cstdint>
#include <functional>
#include <utility>

namespace robosense
{
namespace lidar
{

std::string getDriverVersion();

/**
 * @brief This is the RoboSense LiDAR driver interface class
 */
template <typename T_PointCloud>
class LidarDriver
{
public:

  /**
   * @brief Constructor, instanciate the driver pointer
   */
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

  /**
   * @brief Register the lidar point cloud callback function to driver. When point cloud is ready, this function will be
   * called
   * @param callback The callback function
   */
  inline void regPointCloudCallback(const std::function<std::shared_ptr<T_PointCloud>(void)>& cb_get_cloud,
      const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_put_cloud)
  {
    driver_ptr_->regPointCloudCallback(cb_get_cloud, cb_put_cloud);
  }

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

  inline bool consumePointCloud(const std::shared_ptr<T_PointCloud>& cloud, unsigned int usec = 1000000)
  {
    if (!cloud)
    {
      return false;
    }

    std::shared_ptr<T_PointCloud> ready_cloud;
    if (!getPointCloud(ready_cloud, usec))
    {
      return false;
    }

    *cloud = std::move(*ready_cloud);
    recyclePointCloud(ready_cloud);
    return true;
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
  /**
   * @brief Register the imu data callback function to driver. When imu data is ready, this function will be
   * called
   * @param callback The callback function
   */
  inline void regImuDataCallback(const std::function<std::shared_ptr<ImuData>(void)>& cb_get_imu_data, const std::function<void(const std::shared_ptr<ImuData> &)>& cb_put_imu_data)
  {
    driver_ptr_->regImuDataCallback(cb_get_imu_data, cb_put_imu_data);
  }


  /**
   * @brief Register the lidar difop packet message callback function to driver. When lidar difop packet message is
   * ready, this function will be called
   * @param callback The callback function
   */
  inline void regPacketCallback(const std::function<void(const Packet&)>& cb_put_pkt)
  {
    driver_ptr_->regPacketCallback(cb_put_pkt);
  }

  /**
   * @brief Register the exception message callback function to driver. When error occurs, this function will be called
   * @param callback The callback function
   */
  inline void regExceptionCallback(const std::function<void(const Error&)>& cb_excep)
  {
    cb_exception_ = cb_excep ? cb_excep :
        std::bind(&LidarDriver<T_PointCloud>::defaultExceptionHandler, this, std::placeholders::_1);
    driver_ptr_->regExceptionCallback(cb_exception_);
  }

  /**
   * @brief The initialization function, used to set up parameters and instance objects,
   *        used when get packets from online lidar or pcap
   * @param param The custom struct RSDriverParam
   * @return If successful, return true; else return false
   */
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

  /**
   * @brief Start the thread to receive and decode packets
   * @return If successful, return true; else return false
   */
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

  /**
   * @brief Decode lidar msop/difop messages
   * @param pkt_msg The lidar msop/difop packet
   */
  inline void decodePacket(const Packet& pkt)
  {
    driver_ptr_->decodePacket(pkt);
  }

  /**
   * @brief Get the current lidar temperature
   * @param temp The variable to store lidar temperature
   * @return if get temperature successfully, return true; else return false
   */
  inline bool getTemperature(float& temp)
  {
    return driver_ptr_->getTemperature(temp);
  }

  /**
   * @brief Get device info
   * @param info The variable to store device info
   * @return if get device info successfully, return true; else return false
   */
  inline bool getDeviceInfo(DeviceInfo& info)
  {
    return driver_ptr_->getDeviceInfo(info);
  }

  /**
   * @brief Get device status
   * @param info The variable to store device status
   * @return if get device info successfully, return true; else return false
   */
  inline bool getDeviceStatus(DeviceStatus& status)
  {
    return driver_ptr_->getDeviceStatus(status);
  }

  /**
   * @brief Stop all threads
   */
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
  inline std::shared_ptr<T_PointCloud> waitPointCloud(unsigned int usec = 1000000)
  {
    std::shared_ptr<T_PointCloud> ready_cloud;
    getPointCloud(ready_cloud, usec);
    return ready_cloud;
  }

  inline void releasePointCloud(const std::shared_ptr<T_PointCloud>& cloud)
  {
    if (!cloud)
    {
      return;
    }

    pushFreePointCloud(cloud);
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
  std::shared_ptr<LidarDriverImpl<T_PointCloud>> driver_ptr_;  ///< The driver pointer
};

}  // namespace lidar
}  // namespace robosense
