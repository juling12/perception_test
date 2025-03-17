/*
 * @Author: juling julinger.qq.com
 * @Date: 2024-09-13 10:34:16
 * @LastEditors: juling julinger.qq.com
 * @LastEditTime: 2025-03-17 14:05:45
 */
#include "hz_downsampler.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <iostream>
HzDownSampler ads(10);
ros::Publisher pub;
void handleMsg(const boost::shared_ptr<const sensor_msgs::LaserScan> &msg) {
  if (ads.Pulse()) {
    pub.publish(msg);
    ROS_INFO("publish");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "auto_downsample_test");
  ros::start();
  ros::NodeHandle nh;
  pub = nh.advertise<std_msgs::String>("/auto_downsample", 1);
  ros::Subscriber sub = nh.subscribe("/scan_emma_nav_front", 1, &::handleMsg);

  ros::spin();
  return 0;
}
