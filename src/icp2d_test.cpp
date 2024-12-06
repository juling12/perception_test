/*
 * @Author: juling julinger.qq.com
 * @Date: 2024-11-06 09:53:36
 * @LastEditors: juling julinger.qq.com
 * @LastEditTime: 2024-11-06 10:56:17
 */
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/core.hpp>

int main(int argc, char **argv) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud->width = 8;
  cloud->height = 1;
  cloud->is_dense = true;
  cloud->points.resize(cloud->width * cloud->height);
  // for (auto &point : cloud->points) {
  //   point.x = 1024 * rand() / (RAND_MAX + 1.0f);
  //   point.y = 1024 * rand() / (RAND_MAX + 1.0f);
  //   point.z = 1024 * rand() / (RAND_MAX + 1.0f);
  // }

  // 两帧源点云
  // cloud->points[0].x = -0.679982;
  // cloud->points[0].y = 0.0544634;
  // cloud->points[0].z = 0;

  // cloud->points[1].x = -0.68313;
  // cloud->points[1].y = 0.122504;
  // cloud->points[1].z = 0;

  // cloud->points[2].x = -0.684357;
  // cloud->points[2].y = 0.171597;
  // cloud->points[2].z = 0;

  // cloud->points[3].x = -0.685862;
  // cloud->points[3].y = 0.220043;
  // cloud->points[3].z = 0;

  // cloud->points[4].x = -0.682312;
  // cloud->points[4].y = 0.0543844;
  // cloud->points[4].z = 0;

  // cloud->points[5].x = -0.6836;
  // cloud->points[5].y = 0.122203;
  // cloud->points[5].z = 0;

  // cloud->points[6].x = -0.684844;
  // cloud->points[6].y = 0.171479;
  // cloud->points[6].z = 0;

  // cloud->points[7].x = -0.682346;
  // cloud->points[7].y = 0.219787;
  // cloud->points[7].z = 0;

  for (int i = 0; i < cloud->points.size(); ++i) {
    auto &point = cloud->points[i];
    // std::cout << point.x << " " << point.y << " " << point.z << std::endl;
    if (i < 4) {
      point.r = 255;
      point.g = 0;
      point.b = 0;
    } else {
      point.r = 0;
      point.g = 255;
      point.b = 0;
    }
  }

  // pcl::visualization::CloudViewer viewer("Viewer");
  // viewer.showCloud(cloud);

  pcl::visualization::PCLVisualizer viewer("Viewer");
  viewer.setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
      cloud);
  viewer.addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "cloud");
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
  viewer.setCameraPosition(0, 0, 10, 0, 0, 0, 0, -1,
                          0);  // 设置相机位置（x, y, z）以及方向（视线朝向）

  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
  }

  return 0;
}