#include <lidar_driver.hpp>

#ifdef ENABLE_PCL_POINTCLOUD
#include <msg/pcl_point_cloud_msg.hpp>
#else
#include <msg/point_cloud_msg.hpp>
#endif

#include <memory>

using namespace robosense::lidar;

using PointT = PointXYZI;
using PointCloudMsg = PointCloudT<PointT>;

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

  if (!driver.init(param))
  {
    RS_ERROR << "Driver Initialize Error..." << RS_REND;
    return -1;
  }

  if (!driver.start())
  {
    RS_ERROR << "Driver Start Error..." << RS_REND;
    return -1;
  }

  RS_DEBUG << "RoboSense Lidar-Driver Linux main start......" << RS_REND;

  while (true)
  {
    std::shared_ptr<PointCloudMsg> data;
    if (!driver.getPointCloud(data))
    {
      continue;
    }

    RS_MSG << "msg: " << data->seq << " point cloud size: " << data->points.size() << RS_REND;
    driver.recyclePointCloud(data);
  }
}
