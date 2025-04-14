#include <glog/logging.h>
#include <pluginlib/class_loader.h>

#include <boost/filesystem.hpp>

#include "common/precision_utils.h"
#include "iplus_perception/precision_test_tool.h"

class PrecisionNode {
  struct IncrementalPoseInfo {
    int frame_seq;
    ros::Time stamp;
    Eigen::Affine3d wMe;
    Eigen::Affine3d baseN2base0;
  };

 public:
  PrecisionNode(ros::NodeHandle &nh);
  ~PrecisionNode();

 private:
  void wallTimeCB(const ::ros::WallTimerEvent &unused_timer_event);

  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<pluginlib::ClassLoader<SVC::Perception>> perception_base_;
  boost::shared_ptr<SVC::Perception> p_camera_, p_marker_;
  ros::WallTimer timer_;

  int camera_type_;
  int tag_type_;
  camTypeConfig cam_type_config_;
  std::map<std::string, Eigen::Affine3d> cMe_map_;

  std::string bag_name_, bag_root_path_, csv_file_path_;
  bool base0_initialized_ = false;
  Eigen::Affine3d first_qrcode_pose_, first_marker_pose_;  // 第一帧作为base0
  int frame_seq_ = 0;
  std::ofstream file_;
};

PrecisionNode::PrecisionNode(ros::NodeHandle &nh)
    : nh_(std::make_shared<ros::NodeHandle>(nh)) {
  // create file
  csv_file_path_ = "/mnt/hgfs/data/test.csv";
  if (!ros::param::get("~csv_file", csv_file_path_)) {
    LOG(WARNING) << "No csv_file specified. default: /mnt/hgfs/data/test.csv";
  }
  LOG(ERROR) << "csv_file_path_: " << csv_file_path_;

  file_.open(csv_file_path_);
  file_ << "Data saved at " << timeToString(ros::Time::now()) << "\n";
  file_ << "All data is in mm and deg\n";
  file_ << "frame_seq, camera_stamp, x(mm), y(mm), yaw(deg), marker_stamp, "
           "x(mm), y(mm), yaw(deg), delta_stamp, error_x, error_y, "
           "error_yaw\n";
  file_.close();

  perception_base_ = std::make_shared<pluginlib::ClassLoader<SVC::Perception>>(
      "iplus_perception", "SVC::Perception");

  // initialize camera perception
  p_camera_ = perception_base_->createInstance("SVC::CameraPerception");

  std::string camera_name = "down";
  std::string tag_name = "2XN";
  bool switch_to_cam3d = false;
  bool replay_bag = false;

  camera_type_ = camName2camType(camera_name);
  tag_type_ = tagName2tagType(tag_name);

  std::string error_string;
  getcMes(camera_type_, cMe_map_, error_string);
  if (cMe_map_.find("default") == cMe_map_.end())
    cMe_map_["default"] = Eigen::Affine3d::Identity();

  cam_type_config_ = loadCamConfig(camera_name);
  LOG(INFO) << "============ cam_type_config_ ===================\n"
            << cam_type_config_;

  std::map<std::string, boost::any> value;
  value["camera_type"] = camera_type_;
  value["tag_type"] = tag_type_;
  value["is_lateral_base"] = false;
  value["switch_to_cam3d"] = switch_to_cam3d;
  value["replay_bag"] = replay_bag;
  value["teaching"] = false;
  value["sample_ratio"] = 1.0f;
  value["cMe_map"] = cMe_map_;
  value["debug_mode"] = true;
  value["absolute_pose"] = true;

  if (!p_camera_->initialize(nh_, value)) {
    LOG(ERROR) << "CameraPerception initialize failed.";
    return;
  }

  // initialize dotmarker perception
  p_marker_ = perception_base_->createInstance("SVC::DotMarkerPerception");
  std::map<std::string, boost::any> config_map;
  config_map["ir_image_topic"] = std::string("/pallet/ir/image_raw");
  config_map["camera_cloud_topic"] = std::string("/pallet/camera/cloud");

  geometry_msgs::PolygonStamped interest_polygon;
  interest_polygon.polygon.points.resize(4);
  if (false) {
    // 10.222叉车bag
    interest_polygon.header.frame_id = "map";
    interest_polygon.polygon.points[0].z =
        interest_polygon.polygon.points[1].z =
            interest_polygon.polygon.points[2].z =
                interest_polygon.polygon.points[3].z = 0.525;

    interest_polygon.polygon.points[0].x = -2.3;
    interest_polygon.polygon.points[0].y = -1.4;

    interest_polygon.polygon.points[1].x = -2;
    interest_polygon.polygon.points[1].y = -1.4;

    interest_polygon.polygon.points[2].x = -2;
    interest_polygon.polygon.points[2].y = -0.2;

    interest_polygon.polygon.points[3].x = -2.3;
    interest_polygon.polygon.points[3].y = -0.2;
  }
  if (true) {
    // 钱江bag
    interest_polygon.header.frame_id = "map";
    interest_polygon.polygon.points[0].z =
        interest_polygon.polygon.points[1].z =
            interest_polygon.polygon.points[2].z =
                interest_polygon.polygon.points[3].z = 0.05;

    interest_polygon.polygon.points[0].x = -0.3;
    interest_polygon.polygon.points[0].y = 0.7;

    interest_polygon.polygon.points[1].x = 0;
    interest_polygon.polygon.points[1].y = 0.7;

    interest_polygon.polygon.points[2].x = 0;
    interest_polygon.polygon.points[2].y = -0.5;

    interest_polygon.polygon.points[3].x = -0.3;
    interest_polygon.polygon.points[3].y = -0.5;
  }
  config_map["interest_polygon"] = interest_polygon;
  config_map["marker_path"] = std::string(
      "/opt/jz/config/carly-web/templates/shelves/marker-test/"
      "marker-test.json");
  config_map["debug_mode"] = true;
  config_map["replay_bag"] = false;

  if (!p_marker_->initialize(nh_, config_map)) {
    LOG(ERROR) << "DotMarkerPerception initialize failed.";
    return;
  }

  // start camera perception and dotmarker perception
  timer_ = nh_->createWallTimer(::ros::WallDuration(1.0 / 20),
                                &PrecisionNode::wallTimeCB, this);
  p_camera_->start();
  p_marker_->start();
}

PrecisionNode::~PrecisionNode() {
  if (p_camera_ != nullptr && p_marker_ != nullptr) {
    p_camera_->stop();
    p_marker_->stop();
    timer_.stop();
  }
}

void PrecisionNode::wallTimeCB(
    const ::ros::WallTimerEvent &unused_timer_event) {
  SVC::PerceptionResult pr1, pr2;
  p_camera_->GetPerceptionByClass(pr1);
  p_marker_->GetPerceptionByClass(pr2);
  if (pr1.valid) LOG(WARNING) << "r1 stamp: " << pr1.stamp;
  if (pr2.valid) LOG(WARNING) << "r2 stamp: " << pr2.stamp;

  if (pr1.valid && pr2.valid) {
    frame_seq_++;
    LOG(ERROR) << "********* frame_seq: " << frame_seq_ << " *********";
    LOG(ERROR) << "camera stamp: " << pr1.stamp;
    LOG(ERROR) << "marker stamp: " << pr2.stamp;
    if (base0_initialized_ == false) {
      first_qrcode_pose_ = pr1.wMe;
      first_marker_pose_ = pr2.wMe;
      base0_initialized_ = true;
      LOG(ERROR) << "first qrcode pose: " << getXYYaw(first_qrcode_pose_);
      LOG(ERROR) << "first marker pose: " << getXYYaw(first_marker_pose_);
    }

    if (!base0_initialized_) return;

    // calculate error
    Eigen::Affine3d tag_baseN2base0 = first_qrcode_pose_.inverse() * pr1.wMe;
    Eigen::Affine3d marker_baseN2base0 = first_marker_pose_.inverse() * pr2.wMe;
    LOG(ERROR) << "tag baseN2base0: " << getXYYaw(tag_baseN2base0);
    LOG(ERROR) << "marker baseN2base0: " << getXYYaw(marker_baseN2base0);
    Eigen::Affine3d error = tag_baseN2base0 * marker_baseN2base0.inverse();
    // LOG(INFO) << "e: \n" << error.matrix();
    LOG(INFO) << "error: x: " << error.translation().x() * 1000
              << "mm, y: " << error.translation().y() * 1000
              << "mm, yaw: " << getYaw(error) * (180 / M_PI) << "deg";

    if (std::abs(error.translation().x()) > 0.015 ||
        std::abs(error.translation().y()) > 0.015 ||
        std::abs(getYaw(error)) * (180 / M_PI) > 2) {
      LOG(WARNING) << "error too large. 15mm or 2deg";
    }
    if (std::abs(error.translation().x()) > 0.01 ||
        std::abs(error.translation().y()) > 0.01 ||
        std::abs(getYaw(error)) * (180 / M_PI) > 1) {
      LOG(WARNING) << "error 10mm or 1deg";
    }
    std::cout << std::endl;

    // save data
    file_.open(csv_file_path_, std::ios_base::app);
    auto wMe_dof1 = getXYYaw(pr1.wMe);
    auto wMe_dof2 = getXYYaw(pr2.wMe);
    auto error_dof = getXYYaw(error);
    file_ << frame_seq_ << ", ";
    file_ << std::fixed << std::setprecision(5);
    file_ << timeToDouble(pr1.stamp) << ", ";
    file_ << std::fixed << std::setprecision(4);
    file_ << wMe_dof1.x() * 1000 << ", " << wMe_dof1.y() * 1000 << ", "
          << wMe_dof1.z() << ", ";
    file_ << std::fixed << std::setprecision(5);
    file_ << timeToDouble(pr2.stamp) << ", ";
    file_ << std::fixed << std::setprecision(4);
    file_ << wMe_dof2.x() * 1000 << ", " << wMe_dof2.y() * 1000 << ", "
          << wMe_dof2.z();
    file_ << ", " << (pr1.stamp - pr2.stamp).toSec() << ", "
          << error_dof.x() * 1000 << ", " << error_dof.y() * 1000 << ", "
          << error_dof.z() << std::endl;
    file_.close();
  }
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_logbufsecs = 0;

  ros::init(argc, argv, "precision_node");
  ros::start();
  ros::NodeHandle nh;
  PrecisionNode test(nh);
  LOG(INFO) << "precision_node start!";
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  google::ShutdownGoogleLogging();

  return 0;
}
