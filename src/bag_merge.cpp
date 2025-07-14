#include <glog/logging.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

#include <string>
#include <vector>

int main(int argc, char** argv) {
  ros::init(argc, argv, "bag_merge_cpp11");
  ros::NodeHandle nh("~");

  std::string bag1_path, bag2_path, output_path;
  nh.param<std::string>("bag1", bag1_path, "demo-2025-07-09-15-46-50.bag");
  nh.param<std::string>("bag2", bag2_path, "demo0709-Bbox-now.bag");
  output_path =
      bag1_path.substr(0, bag1_path.find_last_of("/\\") + 1) + "/merged3.bag";

  rosbag::Bag bag1, bag2, outbag;
  bag1.open(bag1_path, rosbag::bagmode::Read);
  bag2.open(bag2_path, rosbag::bagmode::Read);
  outbag.open(output_path, rosbag::bagmode::Write);

  std::vector<rosbag::MessageInstance> color_msgs;
  rosbag::View view1(bag1, rosbag::TopicQuery("/pallet/color/image_raw"));
  for (auto it = view1.begin(); it != view1.end(); ++it) {
    color_msgs.push_back(*it);
  }

  size_t idx = 0;
  rosbag::View view2(bag2, rosbag::TopicQuery("/Bbox"));
  for (auto it = view2.begin(); it != view2.end(); ++it) {
    rosbag::MessageInstance bbox_msg = *it;
    ros::Time t1 = bbox_msg.getTime();

    outbag.write("/Bbox", t1, bbox_msg);

    while (idx < color_msgs.size() && color_msgs[idx].getTime() < t1) {
      ++idx;
    }

    auto msg = color_msgs[idx].instantiate<sensor_msgs::Image>();
    if (idx < color_msgs.size()) {
      msg->header.stamp = t1;
      outbag.write("/pallet/color/image_raw", t1, msg);
    } else {
      msg->header.stamp = ros::Time(0);
      outbag.write("/pallet/color/image_raw", ros::Time(0), msg);
      LOG(ERROR) << "no stamp: " << color_msgs[idx].getTime()
                 << ", idx: " << idx
                 << ", color_msgs.size(): " << color_msgs.size();
    }
  }

  rosbag::View all_view(bag1);
  for (auto it = all_view.begin(); it != all_view.end(); ++it) {
    const std::string& topic = it->getTopic();
    if (topic != "/pallet/color/image_raw") {
      outbag.write(topic, it->getTime(), *it);
    }
  }

  bag1.close();
  bag2.close();
  outbag.close();

  LOG(INFO) << "Merged bag saved to " << output_path.c_str();
  return 0;
}
