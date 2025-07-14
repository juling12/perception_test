/*
 * @Author: juling julinger@qq.com
 * @Date: 2025-07-11 14:57:40
 * @LastEditors: juling julinger@qq.com
 * @LastEditTime: 2025-07-14 20:30:11
 */
#include <glog/logging.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "bag_offset");
  ros::NodeHandle nh("~");

  std::string input_bag_path, output_bag_path;
  double offset_sec = 0.0;

  if (!nh.getParam("input_bag", input_bag_path)) {
    LOG(ERROR) << "Failed to get input_bag parameter.";
    return -1;
  }

  if (!nh.getParam("offset", offset_sec)) {
    LOG(WARNING) << "Param 'offset' not set, using default 0.0";
  }

  ros::Duration offset(offset_sec);
  output_bag_path =
      input_bag_path.substr(0, input_bag_path.find_last_of("/\\") + 1) +
      "offset_" + std::to_string(offset_sec) + ".bag";

  std::vector<std::string> target_topics{"/pallet/color/image_raw",
                                         "/pallet/color/camera_info", "/Bbox"};

  LOG(INFO) << "Reading bag: " << input_bag_path;
  LOG(INFO) << "Writing to: " << output_bag_path;
  LOG(INFO) << "Applying time offset (s): " << offset_sec;

  rosbag::Bag input_bag, output_bag;
  input_bag.open(input_bag_path, rosbag::bagmode::Read);
  output_bag.open(output_bag_path, rosbag::bagmode::Write);

  rosbag::View view(input_bag);

  for (const rosbag::MessageInstance& m : view) {
    bool matched = false;

    for (const auto& topic : target_topics) {
      if (m.getTopic() == topic) {
        matched = true;

        if (topic == "/pallet/color/image_raw") {
          auto msg = m.instantiate<sensor_msgs::Image>();
          if (msg != nullptr) {
            msg->header.stamp += offset;
            output_bag.write(topic, m.getTime(), msg);
          }
        } else if (topic == "/pallet/color/camera_info") {
          auto msg = m.instantiate<sensor_msgs::CameraInfo>();
          if (msg != nullptr) {
            msg->header.stamp += offset;
            output_bag.write(topic, m.getTime(), msg);
          }
        } else if (topic == "/Bbox") {
          auto msg = m.instantiate<jsk_recognition_msgs::BoundingBoxArray>();
          if (msg != nullptr) {
            msg->header.stamp += offset;
            output_bag.write(topic, m.getTime(), msg);
          }
        }

        break;
      }
    }

    if (!matched) {
      output_bag.write(m.getTopic(), m.getTime(), m);
    }
  }

  input_bag.close();
  output_bag.close();
  LOG(INFO) << "Finished writing output bag.";

  return 0;
}
