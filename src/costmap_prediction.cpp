/*
 * @Author: juling julinger@qq.com
 * @Date: 2025-04-07 16:58:16
 * @LastEditors: juling julinger@qq.com
 * @LastEditTime: 2025-04-09 14:18:48
 */
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <glog/logging.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <algorithm>
#include <opencv2/opencv.hpp>

#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

struct PersonState {
  uint id;
  float x;
  float y;
  float vx;
  float vy;
  float height;
  float width;
};

/**
 * @brief 绘制厚度很小的CUBE作为2d的box
 */
visualization_msgs::Marker drawBox2d(
    const std_msgs::Header &header, const float &box_center_x,
    const float &box_center_y, const float &box_theta, const float &box_height,
    const float &box_width, const int &marker_id,
    std_msgs::ColorRGBA marker_color = [] {
      std_msgs::ColorRGBA c;
      c.r = 1.0;
      c.g = 0.0;
      c.b = 0.0;
      c.a = 0.6;
      return c;
    }()) {
  visualization_msgs::Marker box;
  box.header.frame_id = header.frame_id;
  box.header.stamp = header.stamp;
  box.ns = "box2d";
  box.id = marker_id;
  box.type = visualization_msgs::Marker::CUBE;
  box.action = visualization_msgs::Marker::ADD;
  box.pose.position.x = box_center_x;
  box.pose.position.y = box_center_y;
  box.pose.position.z = 0;
  box.pose.orientation = tf::createQuaternionMsgFromYaw(box_theta);
  box.scale.x = box_height;
  box.scale.y = box_width;
  box.scale.z = 0.01;
  box.color = marker_color;
  return box;
}

/**
 * @brief 绘制box的中心点
 */
visualization_msgs::Marker drawBoxCenter(
    const std_msgs::Header &header, const float &center_x,
    const float &center_y, const int &marker_id,
    std_msgs::ColorRGBA marker_color = [] {
      std_msgs::ColorRGBA c;
      c.r = 0.0;
      c.g = 0.0;
      c.b = 1.0;
      c.a = 1.0;
      return c;
    }()) {
  visualization_msgs::Marker center;
  center.header = header;
  center.ns = "box_center";
  center.id = marker_id;
  center.type = visualization_msgs::Marker::SPHERE;
  center.action = visualization_msgs::Marker::ADD;
  center.pose.position.x = center_x;
  center.pose.position.y = center_y;
  center.pose.position.z = 0.01;
  center.scale.x = 0.02;
  center.scale.y = 0.02;
  center.scale.z = 0.01;
  center.color = marker_color;
  return center;
}

/**
 * @brief 绘制箭头
 */
visualization_msgs::Marker drawArrow(
    const std_msgs::Header &header, const geometry_msgs::Point &start_point,
    const geometry_msgs::Point &end_point, const int &marker_id,
    std_msgs::ColorRGBA marker_color = [] {
      std_msgs::ColorRGBA c;
      c.r = 0.0;
      c.g = 1.0;
      c.b = 0.0;
      c.a = 1.0;
      return c;
    }()) {
  visualization_msgs::Marker arrow;
  arrow.header = header;
  arrow.ns = "arrow";
  arrow.id = marker_id;
  arrow.type = visualization_msgs::Marker::ARROW;
  arrow.action = visualization_msgs::Marker::ADD;
  arrow.scale.x = 0.01;
  arrow.scale.y = 0.02;
  arrow.color = marker_color;
  arrow.points.push_back(start_point);
  arrow.points.push_back(end_point);
  return arrow;
}

bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my,
                double origin_x, double origin_y, double resolution,
                int map_width, int map_height) {
  if (wx < origin_x || wy < origin_y) return false;
  mx = (int)((wx - origin_x) / resolution);
  my = (int)((wy - origin_y) / resolution);

  if (mx < map_width && my < map_height) return true;

  return false;
}

/**
 * @brief 计算角点的最大最小值
 */
std::tuple<double, double, double, double> getBoxMinMax(
    const std::vector<geometry_msgs::Point32> &corners) {
  double min_x = corners[0].x, max_x = corners[0].x;
  double min_y = corners[0].y, max_y = corners[0].y;

  for (const auto &pt : corners) {
    min_x = std::min(min_x, static_cast<double>(pt.x));
    max_x = std::max(max_x, static_cast<double>(pt.x));
    min_y = std::min(min_y, static_cast<double>(pt.y));
    max_y = std::max(max_y, static_cast<double>(pt.y));
  }

  return std::make_tuple(min_x, min_y, max_x, max_y);
}

/**
 * @brief 计算角点的最大最小值
 */
std::tuple<double, double, double, double> getBoxMinMax(
    const std::vector<cv::Point2d> &corners) {
  double min_x = corners[0].x, max_x = corners[0].x;
  double min_y = corners[0].y, max_y = corners[0].y;

  for (const auto &pt : corners) {
    min_x = std::min(min_x, pt.x);
    max_x = std::max(max_x, pt.x);
    min_y = std::min(min_y, pt.y);
    max_y = std::max(max_y, pt.y);
  }

  return std::make_tuple(min_x, min_y, max_x, max_y);
}

/**
 * @brief 计算person坐标系下预测区域的四个角点
 */
std::vector<cv::Point2d> getCorners(const PersonState &person_state,
                                    const double &delta_t) {
  auto center_x = person_state.x;
  auto center_y = person_state.y;
  auto theta = atan2(person_state.vy, person_state.vx);
  auto box_width = person_state.width;
  auto box_height = person_state.height;
  auto v = std::hypot(person_state.vx, person_state.vy);

  double w = box_width * 0.5;
  double h = box_height * 0.5;
  auto d = v * delta_t;
  std::vector<cv::Point2d> local_corners = {{w, h}, {-w, h}, {-w, -h}, {w, -h}};
  std::vector<cv::Point2d> pred_local_corners = {
      {w + d, h}, {-w + d, h}, {-w + d, -h}, {w + d, -h}};
  std::vector<cv::Point2d> corners;
  corners.reserve(local_corners.size());
  corners.push_back(pred_local_corners[0]);
  corners.push_back(local_corners[1]);
  corners.push_back(local_corners[2]);
  corners.push_back(pred_local_corners[3]);
  return corners;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "gridMap");
  ros::NodeHandle nh;
  ros::Publisher map_pub =
      nh.advertise<nav_msgs::OccupancyGrid>("/grid_map", 1);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>(
      "/people_status_visualization", 10);
  ros::Publisher polygon_pub =
      nh.advertise<geometry_msgs::PolygonStamped>("predict_region", 1);

  nav_msgs::OccupancyGrid map;
  map.header.frame_id = "odom";
  map.header.stamp = ros::Time::now();
  map.info.resolution = 0.1;  // float32
  map.info.width = 100;       // uint32
  map.info.height = 100;      // uint32
  map.info.origin.position.x = 0.2;
  map.info.origin.position.y = 0.6;
  map.info.origin.position.z = 0.0;
  map.info.origin.orientation.x = 0.0;
  map.info.origin.orientation.y = 0.0;
  map.info.origin.orientation.z = 0.0;
  map.info.origin.orientation.w = 0.0;

  auto size = map.info.width * map.info.height;
  int p[size] = {0};  // [0,100]
  p[0] = 80;
  p[10] = 50;
  p[15] = 100;
  p[20] = 10;
  p[19] = 20;
  p[49 * map.info.height + 49] = 100;
  std::vector<signed char> a(p, p + size);
  map.data = a;

  // odom下的人腿信息
  PersonState person_state;
  person_state.id = 0;
  person_state.x = 0.5;
  person_state.y = 1.0;
  person_state.vx = 0.1;
  person_state.vy = 0.2;
  person_state.height = 0.3;
  person_state.width = 0.1;

  auto center_x = person_state.x;
  auto center_y = person_state.y;
  auto theta = atan2(person_state.vy, person_state.vx);
  auto height = person_state.height;
  auto width = person_state.width;
  auto vx = person_state.vx;
  auto vy = person_state.vy;
  LOG(INFO) << "theta: " << theta << ", theta(degree): " << theta * 180 / M_PI;

  Eigen::Affine3d person2odom = Eigen::Affine3d::Identity();
  person2odom.translation() = Eigen::Vector3d(center_x, center_y, 0.0);
  Eigen::Quaterniond q(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
  person2odom.linear() = q.toRotationMatrix();
  LOG(INFO) << "person2odom trans: " << person2odom.translation().transpose()
            << ", euler: " << person2odom.rotation().eulerAngles(2, 1, 0);

  // 可视化人腿信息
  std_msgs::Header header;
  header.frame_id = "odom";
  header.stamp = ros::Time::now();

  geometry_msgs::Point start_point, end_point;
  start_point.x = center_x;
  start_point.y = center_y;
  start_point.z = 0.01;
  end_point.x = center_x + vx * 1;
  end_point.y = center_y + vy * 1;
  end_point.z = 0.01;
  auto velocity_arrow = drawArrow(header, start_point, end_point, 2);
  auto box2d = drawBox2d(header, center_x, center_y, theta, width, height, 0);
  auto box2d_center = drawBoxCenter(header, center_x, center_y, 1);

  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.push_back(box2d);
  marker_array.markers.push_back(box2d_center);
  marker_array.markers.push_back(velocity_arrow);

  // 预测2s
  double pred_x = center_x + vx * 2;
  double pred_y = center_y + vy * 2;
  LOG(INFO) << "pred_x: " << pred_x << ", pred_y: " << pred_y;

  box2d = drawBox2d(header, pred_x, pred_y, theta, width, height, 3);
  box2d_center = drawBoxCenter(header, pred_x, pred_y, 4);
  marker_array.markers.push_back(box2d);
  marker_array.markers.push_back(box2d_center);

  // 计算边界框范围
  auto corners = getCorners(person_state, 2);

  std::vector<cv::Point2d> odom_corners;
  geometry_msgs::PolygonStamped polygon;
  polygon.header = header;
  for (const auto &pt : corners) {
    Eigen::Vector3d local_pt(pt.x, pt.y, 0.0);
    auto odom_pt = person2odom * local_pt;
    geometry_msgs::Point32 p;
    p.x = odom_pt.x();
    p.y = odom_pt.y();
    p.z = 0.0;
    polygon.polygon.points.push_back(p);
    odom_corners.push_back(cv::Point2d{p.x, p.y});
  }

  auto min_max = getBoxMinMax(odom_corners);
  double min_x = std::get<0>(min_max);
  double min_y = std::get<1>(min_max);
  double max_x = std::get<2>(min_max);
  double max_y = std::get<3>(min_max);
  LOG(INFO) << "min_x: " << min_x << ", min_y: " << min_y
            << ", max_x: " << max_x << ", max_y: " << max_y;

  // 转换为map坐标
  auto resolution = map.info.resolution;
  auto origin_x = map.info.origin.position.x;
  auto origin_y = map.info.origin.position.y;
  auto map_width = map.info.width;
  auto map_height = map.info.height;
  LOG(INFO) << "resolution: " << resolution << ", origin_x: " << origin_x
            << ", origin_y: " << origin_y << ", map_width: " << map_width
            << ", map_height: " << map_height;

  std::vector<cv::Point> polygon_pixels;
  for (const auto &pt : odom_corners) {
    unsigned int mx, my;
    if (worldToMap(pt.x, pt.y, mx, my, origin_x, origin_y, resolution,
                   map_width, map_height)) {
      LOG(INFO) << "mx: " << mx << ", my: " << my;
      polygon_pixels.push_back(cv::Point(mx, my));
    }
  }

  cv::Mat map_img(map_height, map_width, CV_8UC1, cv::Scalar(0));
  for (int i = 0; i < map_height; ++i) {
    for (int j = 0; j < map_width; ++j) {
      int idx = i * map_width + j;
      map_img.at<uchar>(i, j) =
          (map.data[idx] >= 0) ? static_cast<uchar>(map.data[idx]) : 0;
    }
  }
  cv::fillPoly(map_img, std::vector<std::vector<cv::Point>>{polygon_pixels},
               255);

  map.data.resize(map_width * map_height);
  for (int i = 0; i < map_height; ++i) {
    for (int j = 0; j < map_width; ++j) {
      int idx = i * map_width + j;
      if (map_img.at<uchar>(i, j) == 255) {
        map.data[idx] = 100;
      }
    }
  }

  while (ros::ok()) {
    map_pub.publish(map);
    marker_pub.publish(marker_array);
    polygon_pub.publish(polygon);
  }

  return 0;
}
