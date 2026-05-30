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

#include <rs_driver/api/lidar_driver.hpp>

#ifdef ENABLE_PCL_POINTCLOUD
#include <rs_driver/msg/pcl_point_cloud_msg.hpp>
#else
#include <rs_driver/msg/point_cloud_msg.hpp>
#endif

using namespace robosense::lidar;

using PointT = PointXYZI;
using PointCloudMsg = PointCloudT<PointT>;

namespace
{

SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

std::shared_ptr<PointCloudMsg> getPointCloud()
{
  std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
  return msg ? msg : std::make_shared<PointCloudMsg>();
}

void putPointCloud(std::shared_ptr<PointCloudMsg> msg)
{
  stuffed_cloud_queue.push(msg);
}

void exceptionCallback(const Error& code)
{
  RS_WARNING << code.toString() << RS_REND;
}

void processCloud()
{
  while (true)
  {
    std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();
    if (!msg)
    {
      continue;
    }

    RS_MSG << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << RS_REND;
    free_cloud_queue.push(msg);
  }
}

}  // namespace

int main(int argc, char* argv[])
{
  (void)argc;
  (void)argv;

  RS_TITLE << "------------------------------------------------------" << RS_REND;
  RS_TITLE << "            RS_Driver Core Version: v" << getDriverVersion() << RS_REND;
  RS_TITLE << "------------------------------------------------------" << RS_REND;

  RSDriverParam param;
  param.input_type = InputType::ONLINE_LIDAR;
  param.lidar_type = LidarType::RSHELIOS;
  param.input_param.msop_port = 6699;
  param.input_param.difop_port = 7788;
  param.print();
   
  LidarDriver<PointCloudMsg> driver;
  driver.regPointCloudCallback(getPointCloud, putPointCloud);
  driver.regExceptionCallback(exceptionCallback);

  if (!driver.init(param))
  {
    RS_ERROR << "Driver Initialize Error..." << RS_REND;
    return -1;
  }

  std::thread cloud_handle_thread(processCloud);
  cloud_handle_thread.detach();

  driver.start();
  RS_DEBUG << "RoboSense Lidar-Driver Linux online demo start......" << RS_REND;

  while (true)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}
