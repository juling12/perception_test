#include <glog/logging.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

#include <string>
#include <vector>

int main(int argc, char** argv) {
  ros::init(argc, argv, "bag_merge_simple");
  ros::NodeHandle nh("~");

  std::string bag1_path, bag2_path, output_path;
  bag1_path =
      "/mnt/hgfs/data/develop/costmap/vision/demo0709/"
      "demo-2025-07-09-15-46-50.bag";
  // bag2_path = "/mnt/hgfs/data/develop/costmap/vision/demo0709/4-Bbox.bag";
  bag2_path =
      "/mnt/hgfs/data/develop/costmap/vision/demo0709/4-offset_0.400000.bag";
  // bag2_path =
  //     "/mnt/hgfs/data/develop/costmap/vision/demo0709/5-Bbox-use_time_now.bag";
  output_path =
      bag1_path.substr(0, bag1_path.find_last_of("/\\") + 1) + "5-merged.bag";

  rosbag::Bag bag1, bag2, outbag;
  bag1.open(bag1_path, rosbag::bagmode::Read);
  bag2.open(bag2_path, rosbag::bagmode::Read);
  outbag.open(output_path, rosbag::bagmode::Write);

  LOG(INFO) << "Merging bag1: " << bag1_path;
  rosbag::View view1(bag1);
  // for (auto it = view1.begin(); it != view1.end(); ++it) {
  //   outbag.write(it->getTopic(), it->getTime(), *it);
  // }

  LOG(INFO) << "Merging bag2: " << bag2_path;
  rosbag::View view2(bag2);
  // for (auto it = view2.begin(); it != view2.end(); ++it) {
  //   auto bbox_msg =
  //   it->instantiate<jsk_recognition_msgs::BoundingBoxArray>(); if (bbox_msg)
  //   {
  //     std::cout << "bbox_msg: " << bbox_msg->header.stamp
  //               << ", bag time: " << it->getTime()
  //               << ", topic: " << it->getTopic() << std::endl;
  //   }
  //   outbag.write(it->getTopic(), it->getTime(), *it);
  // }

  // 查看/Bbox的时间戳
  for (auto it = view2.begin(); it != view2.end(); ++it) {
    auto bbox_msg = it->instantiate<jsk_recognition_msgs::BoundingBoxArray>();
    if (bbox_msg) {
      std::cout << "bbox_msg: " << bbox_msg->header.stamp
                << ", bag time: " << it->getTime()
                << ", topic: " << it->getTopic() << std::endl;
    }
  }

  // 查看激光时间戳
  // for (auto it = view1.begin(); it != view1.end(); ++it) {
  //   auto bbox_msg = it->instantiate<jsk_recognition_msgs::BoundingBoxArray>();
  //   if (bbox_msg) {
  //     std::cout << "bbox_msg: " << bbox_msg->header.stamp
  //               << ", bag time: " << it->getTime()
  //               << ", topic: " << it->getTopic() << std::endl;
  //   }
  // }

  bag1.close();
  bag2.close();
  outbag.close();

  // LOG(INFO) << "Merged bag saved to: " << output_path;
}