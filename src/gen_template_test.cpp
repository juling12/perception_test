#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visual_servo_msgs/TemplateGenerator.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <glog/logging.h>

using namespace std;
using namespace cv;
using namespace ros;

#define OFFSET_X -0.75f
#define OFFSET_Y 0.25f
#define RANGE_X 0.24f
#define RANGE_Y 0.5f
#define RESOLUTION 0.002f

enum ModelType {
  RECT_4_FOOT = 1,
  RECT_8_FOOT,
  CIRCLE_4_FOOT,
  CIRCLE_8_FOOT,
  RECT_2_FOOT,
  CIRCLE_2_FOOT
};

Mat template_shelf() {
  Point p1, p2, p3, p4, p5, p6, p7, p8;
  int board_size = 25;
  p1.x = board_size;
  p1.y = board_size;
  p2.x = board_size;
  p2.y = board_size + (int)(0.112 / RESOLUTION + 0.5);
  p3.x = board_size;
  p3.y = board_size + (int)(1.385 / RESOLUTION + 0.5);
  p4.x = board_size;
  p4.y = board_size + (int)(1.470 / RESOLUTION + 0.5);
  p5.x = board_size + (int)(0.811 / RESOLUTION + 0.5);
  p5.y = board_size;
  p6.x = board_size + (int)(0.811 / RESOLUTION + 0.5);
  p6.y = board_size + (int)(0.112 / RESOLUTION + 0.5);
  p7.x = board_size + (int)(0.811 / RESOLUTION + 0.5);
  p7.y = board_size + (int)(1.385 / RESOLUTION + 0.5);
  p8.x = board_size + (int)(0.811 / RESOLUTION + 0.5);
  p8.y = board_size + (int)(1.470 / RESOLUTION + 0.5);

  int width = p8.x + board_size;
  int height = p8.y + board_size;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  int radius = 0.015 / RESOLUTION + 0.5;
  circle(model, p1, radius, Scalar(255), 2);
  circle(model, p2, radius, Scalar(255), 2);
  circle(model, p3, radius, Scalar(255), 2);
  circle(model, p4, radius, Scalar(255), 2);
  circle(model, p5, radius, Scalar(255), 2);
  circle(model, p6, radius, Scalar(255), 2);
  circle(model, p7, radius, Scalar(255), 2);
  circle(model, p8, radius, Scalar(255), 2);

  // line(model, p1, p2, Scalar(255));
  // line(model, p2, p3, Scalar(255));
  // line(model, p3, p4, Scalar(255));
  GaussianBlur(model, model, Size(0, 0), 5);
  model = model * 255 / 81;

  return model.clone();
}

Mat template_shelf_square() {
  Point p1, p2, p3, p4;
  int board_size = 18;
  p1.x = board_size;
  p1.y = board_size;
  p2.x = board_size;
  p2.y = board_size + (int)(0.925 / RESOLUTION + 0.5);
  p3.x = board_size + (int)(0.925 / RESOLUTION + 0.5);
  p3.y = board_size;
  p4.x = board_size + (int)(0.925 / RESOLUTION + 0.5);
  ;
  p4.y = board_size + (int)(0.925 / RESOLUTION + 0.5);

  int width = p3.x + board_size;
  int height = p4.y + board_size;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  int radius = 0.028 / RESOLUTION + 0.5;
  circle(model, p1, radius, Scalar(255), -1);
  circle(model, p2, radius, Scalar(255), -1);
  circle(model, p3, radius, Scalar(255), -1);
  circle(model, p4, radius, Scalar(255), -1);
  // line(model, p1, p2, Scalar(255));
  // line(model, p2, p3, Scalar(255));
  // line(model, p3, p4, Scalar(255));
  GaussianBlur(model, model, Size(2 * radius + 1, 2 * radius + 1), 5);
  model = model * 255 / 81;

  return model.clone();
}
Mat template_shelf_square2() {
  Point p1, p2, p3, p4;
  int board_size = 20;
  p1.x = board_size;
  p1.y = board_size;
  p2.x = board_size;
  p2.y = board_size + (int)(1.07 / RESOLUTION + 0.5);
  p3.x = board_size + (int)(0.72 / RESOLUTION + 0.5);
  p3.y = board_size;
  p4.x = board_size + (int)(0.72 / RESOLUTION + 0.5);
  p4.y = board_size + (int)(1.07 / RESOLUTION + 0.5);

  int width = p3.x + board_size;
  int height = p4.y + board_size;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  int radius = 0.014 / RESOLUTION + 0.5;
  circle(model, p1, radius, Scalar(255), 5, -1);
  circle(model, p2, radius, Scalar(255), 5, -1);
  circle(model, p3, radius, Scalar(255), 5, -1);
  circle(model, p4, radius, Scalar(255), 5, -1);

  // line(model, p1, p2, Scalar(255));
  // line(model, p2, p3, Scalar(255));
  // line(model, p3, p4, Scalar(255));
  GaussianBlur(model, model, Size(0, 0), 55);
  model = model * 255 / 81;

  return model.clone();
}

Mat template_shelf_square4() {
  Point p1, p2, p3, p4;
  int board_size = 20;
  p1.x = board_size;
  p1.y = board_size;
  p2.x = board_size;
  p2.y = board_size + (int)(1.07 / RESOLUTION + 0.5);
  p3.x = board_size + (int)(0.716 / RESOLUTION + 0.5);
  p3.y = board_size;
  p4.x = board_size + (int)(0.716 / RESOLUTION + 0.5);
  p4.y = board_size + (int)(1.07 / RESOLUTION + 0.5);

  int width = p3.x + board_size;
  int height = p4.y + board_size;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  int radius = 0.014 / RESOLUTION + 0.5;
  circle(model, p1, radius, Scalar(255), 2);
  circle(model, p2, radius, Scalar(255), 2);
  circle(model, p3, radius, Scalar(255), 2);
  circle(model, p4, radius, Scalar(255), 2);

  // line(model, p1, p2, Scalar(255));
  // line(model, p2, p3, Scalar(255));
  // line(model, p3, p4, Scalar(255));
  GaussianBlur(model, model, Size(0, 0), 5);
  model = model * 255 / 81;

  return model.clone();
}

Mat template_shelf_square_model6_1200x798x25() {
  Point p1, p2, p3, p4;
  int board_size = 20;
  p1.x = board_size;
  p1.y = board_size;
  p2.x = board_size;
  p2.y = board_size + (int)(1.2 / RESOLUTION + 0.5);
  p3.x = board_size + (int)(0.798 / RESOLUTION + 0.5);
  p3.y = board_size;
  p4.x = board_size + (int)(0.798 / RESOLUTION + 0.5);
  p4.y = board_size + (int)(1.2 / RESOLUTION + 0.5);

  int width = p3.x + board_size;
  int height = p4.y + board_size;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  int radius = 0.0125 / RESOLUTION + 0.5;
  circle(model, p1, radius, Scalar(255), 2);
  circle(model, p2, radius, Scalar(255), 2);
  circle(model, p3, radius, Scalar(255), 2);
  circle(model, p4, radius, Scalar(255), 2);

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);

  return model.clone();
}

Mat template_shelf_square_model7_982x712x28() {
  Point p1, p2, p3, p4;
  int board_size = 20;
  p1.x = board_size;
  p1.y = board_size;
  p2.x = board_size;
  p2.y = board_size + (int)(0.982 / RESOLUTION + 0.5);
  p3.x = board_size + (int)(0.712 / RESOLUTION + 0.5);
  p3.y = board_size;
  p4.x = board_size + (int)(0.712 / RESOLUTION + 0.5);
  p4.y = board_size + (int)(0.982 / RESOLUTION + 0.5);

  int width = p3.x + board_size;
  int height = p4.y + board_size;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  int radius = 0.014 / RESOLUTION + 0.5;
  circle(model, p1, radius, Scalar(255), 2);
  circle(model, p2, radius, Scalar(255), 2);
  circle(model, p3, radius, Scalar(255), 2);
  circle(model, p4, radius, Scalar(255), 2);

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);

  return model.clone();
}

Mat template_shelf_square_model8_1075x575x25() {
  Point p1, p2, p3, p4;
  int board_size = 20;
  p1.x = board_size;
  p1.y = board_size;
  p2.x = board_size;
  p2.y = board_size + (int)(1.075 / RESOLUTION + 0.5);
  p3.x = board_size + (int)(0.575 / RESOLUTION + 0.5);
  p3.y = board_size;
  p4.x = board_size + (int)(0.575 / RESOLUTION + 0.5);
  p4.y = board_size + (int)(1.075 / RESOLUTION + 0.5);

  int width = p3.x + board_size;
  int height = p4.y + board_size;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  int radius = 0.0125 / RESOLUTION + 0.5;
  circle(model, p1, radius, Scalar(255), 2);
  circle(model, p2, radius, Scalar(255), 2);
  circle(model, p3, radius, Scalar(255), 2);
  circle(model, p4, radius, Scalar(255), 2);

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);

  return model.clone();
}

Mat template_shelf_square_model9_1500x550x100x50() {
  Point p1, p2, p3, p4;
  int board_size = 25;
  int half_rect_width = 0.025 / RESOLUTION + 0.5;
  int half_rect_height = 0.05 / RESOLUTION + 0.5;

  p1.x = board_size + half_rect_width;
  p1.y = board_size + half_rect_height;
  p2.x = board_size + half_rect_width;
  p2.y = board_size + half_rect_height + (int)(1.5 / RESOLUTION + 0.5);
  p3.x = board_size + half_rect_width + (int)(0.55 / RESOLUTION + 0.5);
  p3.y = board_size + half_rect_height;
  p4.x = board_size + half_rect_width + (int)(0.55 / RESOLUTION + 0.5);
  p4.y = board_size + half_rect_height + (int)(1.5 / RESOLUTION + 0.5);

  int width = p3.x + board_size + half_rect_width;
  int height = p4.y + board_size + +half_rect_height;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  //  int radius = 0.0125 / RESOLUTION + 0.5;
  //  circle(model, p1, radius, Scalar(255), 2);
  //  circle(model, p2, radius, Scalar(255), 2);
  //  circle(model, p3, radius, Scalar(255), 2);
  //  circle(model, p4, radius, Scalar(255), 2);

  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
           cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Point(p1.x + half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
           cv::Scalar(255));
  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x + half_rect_width, p2.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
  // cv::Point(p2.x +
  // half_rect_width, p2.y + half_rect_height), cv::Scalar(255));

  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
           cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Point(p3.x + half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x - half_rect_width, p4.y + half_rect_height),
           cv::Scalar(255));
  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x + half_rect_width, p4.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p4.x - half_rect_width, p4.y + half_rect_height),
  // cv::Point(p4.x +
  // half_rect_width, p4.y + half_rect_height), cv::Scalar(255));

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);
  return model.clone();
}

// 1300*800*50*50mm, 最大外径与矩形长宽
Mat template_shelf_square_model10_1250x750x50x50() {
  Point p1, p2, p3, p4;
  int board_size = 25;
  int half_rect_width = 0.025 / RESOLUTION + 0.5;
  int half_rect_height = 0.025 / RESOLUTION + 0.5;

  p1.x = board_size + half_rect_width;
  p1.y = board_size + half_rect_height;
  p2.x = board_size + half_rect_width;
  p2.y = board_size + half_rect_height + (int)(1.25 / RESOLUTION + 0.5);
  p3.x = board_size + half_rect_width + (int)(0.75 / RESOLUTION + 0.5);
  p3.y = board_size + half_rect_height;
  p4.x = board_size + half_rect_width + (int)(0.75 / RESOLUTION + 0.5);
  p4.y = board_size + half_rect_height + (int)(1.25 / RESOLUTION + 0.5);

  int width = p3.x + board_size + half_rect_width;
  int height = p4.y + board_size + +half_rect_height;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  //  int radius = 0.0125 / RESOLUTION + 0.5;
  //  circle(model, p1, radius, Scalar(255), 2);
  //  circle(model, p2, radius, Scalar(255), 2);
  //  circle(model, p3, radius, Scalar(255), 2);
  //  circle(model, p4, radius, Scalar(255), 2);

  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
           cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Point(p1.x + half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
           cv::Scalar(255));
  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x + half_rect_width, p2.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
  // cv::Point(p2.x +
  // half_rect_width, p2.y + half_rect_height), cv::Scalar(255));

  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
           cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Point(p3.x + half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x - half_rect_width, p4.y + half_rect_height),
           cv::Scalar(255));
  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x + half_rect_width, p4.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p4.x - half_rect_width, p4.y + half_rect_height),
  // cv::Point(p4.x +
  // half_rect_width, p4.y + half_rect_height), cv::Scalar(255));

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);
  return model.clone();
}

// 中兴8个脚
// 884.5 * 752 * *38 * 76.5 (width * height)
Mat template_shelf_square_model11() {
  Point p1, p2, p3, p4, p5, p6, p7, p8;
  int board_size = 25;
  int half_rect_width = 0.019 / RESOLUTION + 0.5;
  int half_rect_height = 0.03825 / RESOLUTION + 0.5;

  // outer rectangle, p1~p4 all rectangle certers
  p1.x = board_size + half_rect_width;
  p1.y = board_size + half_rect_height;
  p2.x = board_size + half_rect_width;
  p2.y = board_size + half_rect_height + (int)(0.8845 / RESOLUTION + 0.5);
  p3.x = board_size + half_rect_width + (int)(0.752 / RESOLUTION + 0.5);
  p3.y = board_size + half_rect_height;
  p4.x = board_size + half_rect_width + (int)(0.752 / RESOLUTION + 0.5);
  p4.y = board_size + half_rect_height + (int)(0.8845 / RESOLUTION + 0.5);

  int width = p3.x + board_size + half_rect_width;
  int height = p4.y + board_size + +half_rect_height;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
           cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Point(p1.x + half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
           cv::Scalar(255));
  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x + half_rect_width, p2.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
  // cv::Point(p2.x +
  // half_rect_width, p2.y + half_rect_height), cv::Scalar(255));

  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
           cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Point(p3.x + half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x - half_rect_width, p4.y + half_rect_height),
           cv::Scalar(255));
  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x + half_rect_width, p4.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p4.x - half_rect_width, p4.y + half_rect_height),
  // cv::Point(p4.x +
  // half_rect_width, p4.y + half_rect_height), cv::Scalar(255));

  // inner rectangle
  p5.x = p1.x + (int)((0.036 + 0.038) / RESOLUTION + 0.5);
  p5.y = p1.y;
  p6.x = p5.x;
  p6.y = p2.y;
  p7.x = p3.x - (int)((0.036 + 0.038) / RESOLUTION + 0.5);
  p7.y = p3.y;
  p8.x = p7.x;
  p8.y = p6.y;

  cv::line(model, cv::Point(p5.x - half_rect_width, p5.y - half_rect_height),
           cv::Point(p5.x - half_rect_width, p5.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p5.x - half_rect_width, p5.y - half_rect_height),
  // cv::Point(p5.x +
  // half_rect_width, p5.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p5.x - half_rect_width, p5.y + half_rect_height),
           cv::Point(p5.x + half_rect_width, p5.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p6.x - half_rect_width, p6.y - half_rect_height),
           cv::Point(p6.x - half_rect_width, p6.y + half_rect_height),
           cv::Scalar(255));
  cv::line(model, cv::Point(p6.x - half_rect_width, p6.y - half_rect_height),
           cv::Point(p6.x + half_rect_width, p6.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p6.x - half_rect_width, p6.y + half_rect_height),
  // cv::Point(p6.x +
  // half_rect_width, p6.y + half_rect_height), cv::Scalar(255));

  cv::line(model, cv::Point(p7.x - half_rect_width, p7.y - half_rect_height),
           cv::Point(p7.x - half_rect_width, p7.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p7.x - half_rect_width, p7.y - half_rect_height),
  // cv::Point(p7.x +
  // half_rect_width, p7.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p7.x - half_rect_width, p7.y + half_rect_height),
           cv::Point(p7.x + half_rect_width, p7.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p8.x - half_rect_width, p8.y - half_rect_height),
           cv::Point(p8.x - half_rect_width, p8.y + half_rect_height),
           cv::Scalar(255));
  cv::line(model, cv::Point(p8.x - half_rect_width, p8.y - half_rect_height),
           cv::Point(p8.x + half_rect_width, p8.y - half_rect_height),
           cv::Scalar(255));

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);
  return model.clone();
}

// 中兴4个脚
// 830.5 * 843.5 * 39.5 * 40.5 (width * height)
Mat template_shelf_square_model12() {
  Point p1, p2, p3, p4;
  int board_size = 25;
  int half_rect_width = (0.0395 / 2.) / RESOLUTION + 0.5;
  int half_rect_height = (0.0405 / 2.) / RESOLUTION + 0.5;

  p1.x = board_size + half_rect_width;
  p1.y = board_size + half_rect_height;
  p2.x = board_size + half_rect_width;
  p2.y = board_size + half_rect_height + (int)(0.8435 / RESOLUTION + 0.5);
  p3.x = board_size + half_rect_width + (int)(0.8305 / RESOLUTION + 0.5);
  p3.y = board_size + half_rect_height;
  p4.x = board_size + half_rect_width + (int)(0.8305 / RESOLUTION + 0.5);
  p4.y = board_size + half_rect_height + (int)(0.8435 / RESOLUTION + 0.5);

  int width = p3.x + board_size + half_rect_width;
  int height = p4.y + board_size + +half_rect_height;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  //  int radius = 0.0125 / RESOLUTION + 0.5;
  //  circle(model, p1, radius, Scalar(255), 2);
  //  circle(model, p2, radius, Scalar(255), 2);
  //  circle(model, p3, radius, Scalar(255), 2);
  //  circle(model, p4, radius, Scalar(255), 2);

  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
           cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Point(p1.x + half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
           cv::Scalar(255));
  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x + half_rect_width, p2.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
  // cv::Point(p2.x +
  // half_rect_width, p2.y + half_rect_height), cv::Scalar(255));

  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
           cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Point(p3.x + half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x - half_rect_width, p4.y + half_rect_height),
           cv::Scalar(255));
  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x + half_rect_width, p4.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p4.x - half_rect_width, p4.y + half_rect_height),
  // cv::Point(p4.x +
  // half_rect_width, p4.y + half_rect_height), cv::Scalar(255));

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);
  return model.clone();
}

// 山东药波演示
// 830*1230*50*50
Mat template_shelf_square_model13() {
  Point p1, p2, p3, p4;
  int board_size = 25;
  int half_rect_width = (0.05 / 2.) / RESOLUTION + 0.5;
  int half_rect_height = (0.05 / 2.) / RESOLUTION + 0.5;

  p1.x = board_size + half_rect_width;
  p1.y = board_size + half_rect_height;
  p2.x = board_size + half_rect_width;
  p2.y = board_size + half_rect_height + (int)(0.83 / RESOLUTION + 0.5);
  p3.x = board_size + half_rect_width + (int)(1.23 / RESOLUTION + 0.5);
  p3.y = board_size + half_rect_height;
  p4.x = board_size + half_rect_width + (int)(1.23 / RESOLUTION + 0.5);
  p4.y = board_size + half_rect_height + (int)(0.83 / RESOLUTION + 0.5);

  int width = p3.x + board_size + half_rect_width;
  int height = p4.y + board_size + +half_rect_height;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  //  int radius = 0.0125 / RESOLUTION + 0.5;
  //  circle(model, p1, radius, Scalar(255), 2);
  //  circle(model, p2, radius, Scalar(255), 2);
  //  circle(model, p3, radius, Scalar(255), 2);
  //  circle(model, p4, radius, Scalar(255), 2);

  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
           cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Point(p1.x + half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
           cv::Scalar(255));
  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x + half_rect_width, p2.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
  // cv::Point(p2.x +
  // half_rect_width, p2.y + half_rect_height), cv::Scalar(255));

  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
           cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Point(p3.x + half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x - half_rect_width, p4.y + half_rect_height),
           cv::Scalar(255));
  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x + half_rect_width, p4.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p4.x - half_rect_width, p4.y + half_rect_height),
  // cv::Point(p4.x +
  // half_rect_width, p4.y + half_rect_height), cv::Scalar(255));

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);
  return model.clone();
}

// 20190528
// 上海联影医疗项目
Mat template_shelf_square_model15() {
  Point p1, p2, p3, p4, p5, p6, p7, p8;
  int board_size = 25;
  int shelf_leg_radius = 0.015 / RESOLUTION + 0.5;

  // outer rectangle, p1~p4 all rectangle certers
  p1.x = board_size + shelf_leg_radius;
  p1.y = board_size + shelf_leg_radius;
  p2.x = board_size + shelf_leg_radius;
  p2.y = board_size + shelf_leg_radius + (int)(0.09 / RESOLUTION + 0.5);
  p3.x = board_size + shelf_leg_radius;
  p3.y =
      board_size + shelf_leg_radius + (int)((0.09 + 0.87) / RESOLUTION + 0.5);
  p4.x = board_size + shelf_leg_radius;
  p4.y = board_size + shelf_leg_radius +
         (int)((0.09 + 0.87 + 0.09) / RESOLUTION + 0.5);

  p5.x = board_size + shelf_leg_radius + (int)(1.39 / RESOLUTION + 0.5);
  p5.y = board_size + shelf_leg_radius;
  p6.x = board_size + shelf_leg_radius + (int)(1.39 / RESOLUTION + 0.5);
  p6.y = board_size + shelf_leg_radius + (int)(0.09 / RESOLUTION + 0.5);
  p7.x = board_size + shelf_leg_radius + (int)(1.39 / RESOLUTION + 0.5);
  p7.y =
      board_size + shelf_leg_radius + (int)((0.09 + 0.87) / RESOLUTION + 0.5);
  p8.x = board_size + shelf_leg_radius + (int)(1.39 / RESOLUTION + 0.5);
  p8.y = board_size + shelf_leg_radius +
         (int)((0.09 + 0.87 + 0.09) / RESOLUTION + 0.5);

  int width = p8.x + board_size + shelf_leg_radius;
  int height = p8.y + board_size + shelf_leg_radius;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  circle(model, p1, shelf_leg_radius, Scalar(255), 2);
  circle(model, p2, shelf_leg_radius, Scalar(255), 2);
  circle(model, p3, shelf_leg_radius, Scalar(255), 2);
  circle(model, p4, shelf_leg_radius, Scalar(255), 2);
  circle(model, p5, shelf_leg_radius, Scalar(255), 2);
  circle(model, p6, shelf_leg_radius, Scalar(255), 2);
  circle(model, p7, shelf_leg_radius, Scalar(255), 2);
  circle(model, p8, shelf_leg_radius, Scalar(255), 2);

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);
  return model.clone();
}

// 20190531
// 上海联影医疗项目,8个脚，之后但凡生成4个或8个脚的货架，都以这个为模板，无论是圆形脚还是矩形脚,并去p8之后的点
Mat template_shelf_square_model16() {
  Point p1, p2, p3, p4, p5, p6, p7, p8;
  int board_size = 25;
  int half_rect_width = 0.015 / RESOLUTION + 0.5;
  int half_rect_height = 0.015 / RESOLUTION + 0.5;

  // outer rectangle, p1~p4 all rectangle certers
  p1.x = board_size + half_rect_width;
  p1.y = board_size + half_rect_height;

  p2.x = p1.x;
  p2.y = p1.y + 2 * half_rect_height + (int)(0.055 / RESOLUTION + 0.5);

  p3.x = p1.x;
  p3.y = p2.y + 2 * half_rect_height + (int)(0.8 / RESOLUTION + 0.5);

  p4.x = p1.x;
  p4.y = p3.y + 2 * half_rect_height + (int)(0.055 / RESOLUTION + 0.5);

  p5.x = p1.x + 2 * half_rect_width + (int)(1.94 / RESOLUTION + 0.5);
  p5.y = p1.y;

  p6.x = p2.x + 2 * half_rect_width + (int)(1.94 / RESOLUTION + 0.5);
  p6.y = p2.y;

  p7.x = p3.x + 2 * half_rect_width + (int)(1.94 / RESOLUTION + 0.5);
  p7.y = p3.y;

  p8.x = p4.x + 2 * half_rect_width + (int)(1.94 / RESOLUTION + 0.5);
  p8.y = p4.y;

  Point p1_1, p1_2, p1_3, p4_1, p4_2, p4_3;
  p1_1.x = p1.x + (int)(0.24625 * 3 / RESOLUTION + 0.5);
  p1_1.y = p1.y;

  p1_2.x = p1.x + (int)(0.24625 * 4 / RESOLUTION + 0.5);
  p1_2.y = p1.y;

  p1_3.x = p1.x + (int)(0.24625 * 5 / RESOLUTION + 0.5);
  p1_3.y = p1.y;

  p4_1.x = p4.x + (int)(0.24625 * 3 / RESOLUTION + 0.5);
  p4_1.y = p4.y;

  p4_2.x = p4.x + (int)(0.24625 * 4 / RESOLUTION + 0.5);
  p4_2.y = p4.y;

  p4_3.x = p4.x + (int)(0.24625 * 5 / RESOLUTION + 0.5);
  p4_3.y = p4.y;

  int width = p8.x + board_size + half_rect_width;
  int height = p8.y + board_size + +half_rect_height;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
           cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Point(p1.x + half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
           cv::Point(p2.x + half_rect_width, p2.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
           cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
           cv::Point(p3.x + half_rect_width, p3.y - half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x - half_rect_width, p4.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x + half_rect_width, p4.y - half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p5.x - half_rect_width, p5.y - half_rect_height),
           cv::Point(p5.x - half_rect_width, p5.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p5.x - half_rect_width, p5.y - half_rect_height),
  // cv::Point(p5.x +
  // half_rect_width, p5.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p5.x - half_rect_width, p5.y + half_rect_height),
           cv::Point(p5.x + half_rect_width, p5.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p6.x - half_rect_width, p6.y - half_rect_height),
           cv::Point(p6.x - half_rect_width, p6.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p5.x - half_rect_width, p5.y - half_rect_height),
  // cv::Point(p5.x +
  // half_rect_width, p5.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p6.x - half_rect_width, p6.y + half_rect_height),
           cv::Point(p6.x + half_rect_width, p6.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p7.x - half_rect_width, p7.y - half_rect_height),
           cv::Point(p7.x - half_rect_width, p7.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p7.x - half_rect_width, p7.y - half_rect_height),
  // cv::Point(p7.x +
  // half_rect_width, p7.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p7.x - half_rect_width, p7.y - half_rect_height),
           cv::Point(p7.x + half_rect_width, p7.y - half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p8.x - half_rect_width, p8.y - half_rect_height),
           cv::Point(p8.x - half_rect_width, p8.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p7.x - half_rect_width, p7.y - half_rect_height),
  // cv::Point(p7.x +
  // half_rect_width, p7.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p8.x - half_rect_width, p8.y - half_rect_height),
           cv::Point(p8.x + half_rect_width, p8.y - half_rect_height),
           cv::Scalar(255));

  cv::line(model,
           cv::Point(p1_1.x - half_rect_width, p1_1.y - half_rect_height),
           cv::Point(p1_1.x - half_rect_width, p1_1.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p9.x - half_rect_width, p9.y - half_rect_height),
  // cv::Point(p9.x +
  // half_rect_width, p9.y - half_rect_height), cv::Scalar(255));
  cv::line(model,
           cv::Point(p1_1.x - half_rect_width, p1_1.y + half_rect_height),
           cv::Point(p1_1.x + half_rect_width, p1_1.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model,
           cv::Point(p1_2.x - half_rect_width, p1_2.y - half_rect_height),
           cv::Point(p1_2.x - half_rect_width, p1_2.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p11.x - half_rect_width, p11.y -
  // half_rect_height), cv::Point(p11.x +
  // half_rect_width, p11.y - half_rect_height), cv::Scalar(255));
  cv::line(model,
           cv::Point(p1_2.x - half_rect_width, p1_2.y + half_rect_height),
           cv::Point(p1_2.x + half_rect_width, p1_2.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model,
           cv::Point(p1_3.x - half_rect_width, p1_3.y - half_rect_height),
           cv::Point(p1_3.x - half_rect_width, p1_3.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p13.x - half_rect_width, p13.y -
  // half_rect_height), cv::Point(p13.x +
  // half_rect_width, p13.y - half_rect_height), cv::Scalar(255));
  cv::line(model,
           cv::Point(p1_3.x - half_rect_width, p1_3.y + half_rect_height),
           cv::Point(p1_3.x + half_rect_width, p1_3.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model,
           cv::Point(p4_1.x - half_rect_width, p4_1.y - half_rect_height),
           cv::Point(p4_1.x - half_rect_width, p4_1.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p13.x - half_rect_width, p13.y -
  // half_rect_height), cv::Point(p13.x +
  // half_rect_width, p13.y - half_rect_height), cv::Scalar(255));
  cv::line(model,
           cv::Point(p4_1.x - half_rect_width, p4_1.y - half_rect_height),
           cv::Point(p4_1.x + half_rect_width, p4_1.y - half_rect_height),
           cv::Scalar(255));

  cv::line(model,
           cv::Point(p4_2.x - half_rect_width, p4_2.y - half_rect_height),
           cv::Point(p4_2.x - half_rect_width, p4_2.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p13.x - half_rect_width, p13.y -
  // half_rect_height), cv::Point(p13.x +
  // half_rect_width, p13.y - half_rect_height), cv::Scalar(255));
  cv::line(model,
           cv::Point(p4_2.x - half_rect_width, p4_2.y - half_rect_height),
           cv::Point(p4_2.x + half_rect_width, p4_2.y - half_rect_height),
           cv::Scalar(255));

  cv::line(model,
           cv::Point(p4_3.x - half_rect_width, p4_3.y - half_rect_height),
           cv::Point(p4_3.x - half_rect_width, p4_3.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p13.x - half_rect_width, p13.y -
  // half_rect_height), cv::Point(p13.x +
  // half_rect_width, p13.y - half_rect_height), cv::Scalar(255));
  cv::line(model,
           cv::Point(p4_3.x - half_rect_width, p4_3.y - half_rect_height),
           cv::Point(p4_3.x + half_rect_width, p4_3.y - half_rect_height),
           cv::Scalar(255));

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);
  return model.clone();
}

// 20190612
// 上海联影医疗项目
Mat template_shelf_square_model17() {
  Point p1, p2, p3, p4, p5, p6, p7, p8;
  int board_size = 25;
  int half_rect_width = 0.015 / RESOLUTION + 0.5;
  int half_rect_height = 0.015 / RESOLUTION + 0.5;

  // outer rectangle, p1~p4 all rectangle certers
  p1.x = board_size + half_rect_width;
  p1.y = board_size + half_rect_height;

  p2.x = p1.x;
  p2.y = p1.y + 2 * half_rect_height + (int)(0.055 / RESOLUTION + 0.5);

  p3.x = p1.x;
  p3.y = p2.y + 2 * half_rect_height + (int)(0.8 / RESOLUTION + 0.5);

  p4.x = p1.x;
  p4.y = p3.y + 2 * half_rect_height + (int)(0.055 / RESOLUTION + 0.5);

  p5.x = p1.x + 2 * half_rect_width + (int)(1.17 / RESOLUTION + 0.5);
  p5.y = p1.y;

  p6.x = p2.x + 2 * half_rect_width + (int)(1.17 / RESOLUTION + 0.5);
  p6.y = p2.y;

  p7.x = p3.x + 2 * half_rect_width + (int)(1.17 / RESOLUTION + 0.5);
  p7.y = p3.y;

  p8.x = p4.x + 2 * half_rect_width + (int)(1.17 / RESOLUTION + 0.5);
  p8.y = p4.y;

  //  Point p1_1, p1_2, p1_3, p4_1, p4_2, p4_3;
  //  p1_1.x = p1.x + ( int )(0.24625 * 3 / RESOLUTION + 0.5);
  //  p1_1.y = p1.y;

  //  p1_2.x = p1.x + ( int )(0.24625 * 4 / RESOLUTION + 0.5);
  //  p1_2.y = p1.y;

  //  p1_3.x = p1.x + ( int )(0.24625 * 5 / RESOLUTION + 0.5);
  //  p1_3.y = p1.y;

  //  p4_1.x = p4.x + ( int )(0.24625 * 3 / RESOLUTION + 0.5);
  //  p4_1.y = p4.y;

  //  p4_2.x = p4.x + ( int )(0.24625 * 4 / RESOLUTION + 0.5);
  //  p4_2.y = p4.y;

  //  p4_3.x = p4.x + ( int )(0.24625 * 5 / RESOLUTION + 0.5);
  //  p4_3.y = p4.y;

  int width = p8.x + board_size + half_rect_width;
  int height = p8.y + board_size + +half_rect_height;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
           cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Point(p1.x + half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
           cv::Point(p2.x + half_rect_width, p2.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
           cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
           cv::Point(p3.x + half_rect_width, p3.y - half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x - half_rect_width, p4.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x + half_rect_width, p4.y - half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p5.x - half_rect_width, p5.y - half_rect_height),
           cv::Point(p5.x - half_rect_width, p5.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p5.x - half_rect_width, p5.y - half_rect_height),
  // cv::Point(p5.x +
  // half_rect_width, p5.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p5.x - half_rect_width, p5.y + half_rect_height),
           cv::Point(p5.x + half_rect_width, p5.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p6.x - half_rect_width, p6.y - half_rect_height),
           cv::Point(p6.x - half_rect_width, p6.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p5.x - half_rect_width, p5.y - half_rect_height),
  // cv::Point(p5.x +
  // half_rect_width, p5.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p6.x - half_rect_width, p6.y + half_rect_height),
           cv::Point(p6.x + half_rect_width, p6.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p7.x - half_rect_width, p7.y - half_rect_height),
           cv::Point(p7.x - half_rect_width, p7.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p7.x - half_rect_width, p7.y - half_rect_height),
  // cv::Point(p7.x +
  // half_rect_width, p7.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p7.x - half_rect_width, p7.y - half_rect_height),
           cv::Point(p7.x + half_rect_width, p7.y - half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p8.x - half_rect_width, p8.y - half_rect_height),
           cv::Point(p8.x - half_rect_width, p8.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p7.x - half_rect_width, p7.y - half_rect_height),
  // cv::Point(p7.x +
  // half_rect_width, p7.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p8.x - half_rect_width, p8.y - half_rect_height),
           cv::Point(p8.x + half_rect_width, p8.y - half_rect_height),
           cv::Scalar(255));

  //  cv::line(model, cv::Point(p1_1.x - half_rect_width, p1_1.y -
  //  half_rect_height),
  //           cv::Point(p1_1.x - half_rect_width, p1_1.y + half_rect_height),
  //           cv::Scalar(255));
  //  // cv::line(model, cv::Point(p9.x - half_rect_width, p9.y -
  //  half_rect_height), cv::Point(p9.x
  //  +
  //  // half_rect_width, p9.y - half_rect_height), cv::Scalar(255));
  //  cv::line(model, cv::Point(p1_1.x - half_rect_width, p1_1.y +
  //  half_rect_height),
  //           cv::Point(p1_1.x + half_rect_width, p1_1.y + half_rect_height),
  //           cv::Scalar(255));

  //  cv::line(model, cv::Point(p1_2.x - half_rect_width, p1_2.y -
  //  half_rect_height),
  //           cv::Point(p1_2.x - half_rect_width, p1_2.y + half_rect_height),
  //           cv::Scalar(255));
  //  // cv::line(model, cv::Point(p11.x - half_rect_width, p11.y -
  //  half_rect_height),
  //  cv::Point(p11.x +
  //  // half_rect_width, p11.y - half_rect_height), cv::Scalar(255));
  //  cv::line(model, cv::Point(p1_2.x - half_rect_width, p1_2.y +
  //  half_rect_height),
  //           cv::Point(p1_2.x + half_rect_width, p1_2.y + half_rect_height),
  //           cv::Scalar(255));

  //  cv::line(model, cv::Point(p1_3.x - half_rect_width, p1_3.y -
  //  half_rect_height),
  //           cv::Point(p1_3.x - half_rect_width, p1_3.y + half_rect_height),
  //           cv::Scalar(255));
  //  // cv::line(model, cv::Point(p13.x - half_rect_width, p13.y -
  //  half_rect_height),
  //  cv::Point(p13.x +
  //  // half_rect_width, p13.y - half_rect_height), cv::Scalar(255));
  //  cv::line(model, cv::Point(p1_3.x - half_rect_width, p1_3.y +
  //  half_rect_height),
  //           cv::Point(p1_3.x + half_rect_width, p1_3.y + half_rect_height),
  //           cv::Scalar(255));

  //  cv::line(model, cv::Point(p4_1.x - half_rect_width, p4_1.y -
  //  half_rect_height),
  //           cv::Point(p4_1.x - half_rect_width, p4_1.y + half_rect_height),
  //           cv::Scalar(255));
  //  // cv::line(model, cv::Point(p13.x - half_rect_width, p13.y -
  //  half_rect_height),
  //  cv::Point(p13.x +
  //  // half_rect_width, p13.y - half_rect_height), cv::Scalar(255));
  //  cv::line(model, cv::Point(p4_1.x - half_rect_width, p4_1.y -
  //  half_rect_height),
  //           cv::Point(p4_1.x + half_rect_width, p4_1.y - half_rect_height),
  //           cv::Scalar(255));

  //  cv::line(model, cv::Point(p4_2.x - half_rect_width, p4_2.y -
  //  half_rect_height),
  //           cv::Point(p4_2.x - half_rect_width, p4_2.y + half_rect_height),
  //           cv::Scalar(255));
  //  // cv::line(model, cv::Point(p13.x - half_rect_width, p13.y -
  //  half_rect_height),
  //  cv::Point(p13.x +
  //  // half_rect_width, p13.y - half_rect_height), cv::Scalar(255));
  //  cv::line(model, cv::Point(p4_2.x - half_rect_width, p4_2.y -
  //  half_rect_height),
  //           cv::Point(p4_2.x + half_rect_width, p4_2.y - half_rect_height),
  //           cv::Scalar(255));

  //  cv::line(model, cv::Point(p4_3.x - half_rect_width, p4_3.y -
  //  half_rect_height),
  //           cv::Point(p4_3.x - half_rect_width, p4_3.y + half_rect_height),
  //           cv::Scalar(255));
  //  // cv::line(model, cv::Point(p13.x - half_rect_width, p13.y -
  //  half_rect_height),
  //  cv::Point(p13.x +
  //  // half_rect_width, p13.y - half_rect_height), cv::Scalar(255));
  //  cv::line(model, cv::Point(p4_3.x - half_rect_width, p4_3.y -
  //  half_rect_height),
  //           cv::Point(p4_3.x + half_rect_width, p4_3.y - half_rect_height),
  //           cv::Scalar(255));

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);
  return model.clone();
}
// 20190613
// 上海联影医疗项目
Mat template_shelf_square_model18() {
  Point p1, p2, p3, p4, p5, p6, p7, p8;
  int board_size = 25;
  int half_rect_width = 0.015 / RESOLUTION + 0.5;
  int half_rect_height = 0.015 / RESOLUTION + 0.5;

  // outer rectangle, p1~p4 all rectangle certers
  p1.x = board_size + half_rect_width;
  p1.y = board_size + half_rect_height;

  p2.x = p1.x;
  p2.y = p1.y + 2 * half_rect_height + (int)(0.055 / RESOLUTION + 0.5);

  p3.x = p1.x;
  p3.y = p2.y + 2 * half_rect_height + (int)(0.8 / RESOLUTION + 0.5);

  p4.x = p1.x;
  p4.y = p3.y + 2 * half_rect_height + (int)(0.055 / RESOLUTION + 0.5);

  p5.x = p1.x + 2 * half_rect_width + (int)(1.47 / RESOLUTION + 0.5);
  p5.y = p1.y;

  p6.x = p2.x + 2 * half_rect_width + (int)(1.47 / RESOLUTION + 0.5);
  p6.y = p2.y;

  p7.x = p3.x + 2 * half_rect_width + (int)(1.47 / RESOLUTION + 0.5);
  p7.y = p3.y;

  p8.x = p4.x + 2 * half_rect_width + (int)(1.47 / RESOLUTION + 0.5);
  p8.y = p4.y;

  //  Point p1_1, p1_2, p1_3, p4_1, p4_2, p4_3;
  //  p1_1.x = p1.x + ( int )(0.24625 * 3 / RESOLUTION + 0.5);
  //  p1_1.y = p1.y;

  //  p1_2.x = p1.x + ( int )(0.24625 * 4 / RESOLUTION + 0.5);
  //  p1_2.y = p1.y;

  //  p1_3.x = p1.x + ( int )(0.24625 * 5 / RESOLUTION + 0.5);
  //  p1_3.y = p1.y;

  //  p4_1.x = p4.x + ( int )(0.24625 * 3 / RESOLUTION + 0.5);
  //  p4_1.y = p4.y;

  //  p4_2.x = p4.x + ( int )(0.24625 * 4 / RESOLUTION + 0.5);
  //  p4_2.y = p4.y;

  //  p4_3.x = p4.x + ( int )(0.24625 * 5 / RESOLUTION + 0.5);
  //  p4_3.y = p4.y;

  int width = p8.x + board_size + half_rect_width;
  int height = p8.y + board_size + +half_rect_height;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
           cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Point(p1.x + half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
           cv::Point(p2.x + half_rect_width, p2.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
           cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
           cv::Point(p3.x + half_rect_width, p3.y - half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x - half_rect_width, p4.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x + half_rect_width, p4.y - half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p5.x - half_rect_width, p5.y - half_rect_height),
           cv::Point(p5.x - half_rect_width, p5.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p5.x - half_rect_width, p5.y - half_rect_height),
  // cv::Point(p5.x +
  // half_rect_width, p5.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p5.x - half_rect_width, p5.y + half_rect_height),
           cv::Point(p5.x + half_rect_width, p5.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p6.x - half_rect_width, p6.y - half_rect_height),
           cv::Point(p6.x - half_rect_width, p6.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p5.x - half_rect_width, p5.y - half_rect_height),
  // cv::Point(p5.x +
  // half_rect_width, p5.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p6.x - half_rect_width, p6.y + half_rect_height),
           cv::Point(p6.x + half_rect_width, p6.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p7.x - half_rect_width, p7.y - half_rect_height),
           cv::Point(p7.x - half_rect_width, p7.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p7.x - half_rect_width, p7.y - half_rect_height),
  // cv::Point(p7.x +
  // half_rect_width, p7.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p7.x - half_rect_width, p7.y - half_rect_height),
           cv::Point(p7.x + half_rect_width, p7.y - half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p8.x - half_rect_width, p8.y - half_rect_height),
           cv::Point(p8.x - half_rect_width, p8.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p7.x - half_rect_width, p7.y - half_rect_height),
  // cv::Point(p7.x +
  // half_rect_width, p7.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p8.x - half_rect_width, p8.y - half_rect_height),
           cv::Point(p8.x + half_rect_width, p8.y - half_rect_height),
           cv::Scalar(255));

  //  cv::line(model, cv::Point(p1_1.x - half_rect_width, p1_1.y -
  //  half_rect_height),
  //           cv::Point(p1_1.x - half_rect_width, p1_1.y + half_rect_height),
  //           cv::Scalar(255));
  //  // cv::line(model, cv::Point(p9.x - half_rect_width, p9.y -
  //  half_rect_height), cv::Point(p9.x
  //  +
  //  // half_rect_width, p9.y - half_rect_height), cv::Scalar(255));
  //  cv::line(model, cv::Point(p1_1.x - half_rect_width, p1_1.y +
  //  half_rect_height),
  //           cv::Point(p1_1.x + half_rect_width, p1_1.y + half_rect_height),
  //           cv::Scalar(255));

  //  cv::line(model, cv::Point(p1_2.x - half_rect_width, p1_2.y -
  //  half_rect_height),
  //           cv::Point(p1_2.x - half_rect_width, p1_2.y + half_rect_height),
  //           cv::Scalar(255));
  //  // cv::line(model, cv::Point(p11.x - half_rect_width, p11.y -
  //  half_rect_height),
  //  cv::Point(p11.x +
  //  // half_rect_width, p11.y - half_rect_height), cv::Scalar(255));
  //  cv::line(model, cv::Point(p1_2.x - half_rect_width, p1_2.y +
  //  half_rect_height),
  //           cv::Point(p1_2.x + half_rect_width, p1_2.y + half_rect_height),
  //           cv::Scalar(255));

  //  cv::line(model, cv::Point(p1_3.x - half_rect_width, p1_3.y -
  //  half_rect_height),
  //           cv::Point(p1_3.x - half_rect_width, p1_3.y + half_rect_height),
  //           cv::Scalar(255));
  //  // cv::line(model, cv::Point(p13.x - half_rect_width, p13.y -
  //  half_rect_height),
  //  cv::Point(p13.x +
  //  // half_rect_width, p13.y - half_rect_height), cv::Scalar(255));
  //  cv::line(model, cv::Point(p1_3.x - half_rect_width, p1_3.y +
  //  half_rect_height),
  //           cv::Point(p1_3.x + half_rect_width, p1_3.y + half_rect_height),
  //           cv::Scalar(255));

  //  cv::line(model, cv::Point(p4_1.x - half_rect_width, p4_1.y -
  //  half_rect_height),
  //           cv::Point(p4_1.x - half_rect_width, p4_1.y + half_rect_height),
  //           cv::Scalar(255));
  //  // cv::line(model, cv::Point(p13.x - half_rect_width, p13.y -
  //  half_rect_height),
  //  cv::Point(p13.x +
  //  // half_rect_width, p13.y - half_rect_height), cv::Scalar(255));
  //  cv::line(model, cv::Point(p4_1.x - half_rect_width, p4_1.y -
  //  half_rect_height),
  //           cv::Point(p4_1.x + half_rect_width, p4_1.y - half_rect_height),
  //           cv::Scalar(255));

  //  cv::line(model, cv::Point(p4_2.x - half_rect_width, p4_2.y -
  //  half_rect_height),
  //           cv::Point(p4_2.x - half_rect_width, p4_2.y + half_rect_height),
  //           cv::Scalar(255));
  //  // cv::line(model, cv::Point(p13.x - half_rect_width, p13.y -
  //  half_rect_height),
  //  cv::Point(p13.x +
  //  // half_rect_width, p13.y - half_rect_height), cv::Scalar(255));
  //  cv::line(model, cv::Point(p4_2.x - half_rect_width, p4_2.y -
  //  half_rect_height),
  //           cv::Point(p4_2.x + half_rect_width, p4_2.y - half_rect_height),
  //           cv::Scalar(255));

  //  cv::line(model, cv::Point(p4_3.x - half_rect_width, p4_3.y -
  //  half_rect_height),
  //           cv::Point(p4_3.x - half_rect_width, p4_3.y + half_rect_height),
  //           cv::Scalar(255));
  //  // cv::line(model, cv::Point(p13.x - half_rect_width, p13.y -
  //  half_rect_height),
  //  cv::Point(p13.x +
  //  // half_rect_width, p13.y - half_rect_height), cv::Scalar(255));
  //  cv::line(model, cv::Point(p4_3.x - half_rect_width, p4_3.y -
  //  half_rect_height),
  //           cv::Point(p4_3.x + half_rect_width, p4_3.y - half_rect_height),
  //           cv::Scalar(255));

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);
  return model.clone();
}

// 20190617
// 无锡微孚
Mat template_shelf_square_model_wxwf() {
  Point p1, p2, p3, p4, p5, p6, p7, p8;
  int board_size = 25;
  int half_rect_width = 0.025 / RESOLUTION + 0.5;
  int half_rect_height = 0.05 / RESOLUTION + 0.5;

  // outer rectangle, p1~p4 all rectangle certers
  p1.x = board_size + half_rect_width;
  p1.y = board_size + half_rect_height;

  p2.x = p1.x;
  p2.y = p1.y + 2 * half_rect_height + (int)(1.30 / RESOLUTION + 0.5);

  p3.x = p1.x + 2 * half_rect_width + (int)(0.5 / RESOLUTION + 0.5);
  p3.y = p1.y;

  p4.x = p2.x + 2 * half_rect_width + (int)(0.5 / RESOLUTION + 0.5);
  p4.y = p2.y;

  int width = p4.x + board_size + half_rect_width;
  int height = p4.y + board_size + +half_rect_height;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
           cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Point(p1.x + half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));

  //  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y -
  //  half_rect_height),
  //           cv::Point(p2.x + half_rect_width, p2.y - half_rect_height),
  //           cv::Scalar(255));
  //  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y -
  //  half_rect_height), cv::Point(p1.x
  //  +
  //  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  //  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y -
  //  half_rect_height),
  //           cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
  //           cv::Scalar(255));

  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
           cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Point(p3.x + half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x + half_rect_width, p4.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x - half_rect_width, p4.y + half_rect_height),
           cv::Scalar(255));

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);
  return model.clone();
}

// 20190618
// 中兴老化柜
// 20190531
// 上海联影医疗项目,8个脚，之后但凡生成4个或8个脚的货架，都以这个为模板，无论是圆形脚还是矩形脚,并去p8之后的点
Mat template_shelf_square_model_zxlhg() {
  Point p1, p2, p3, p4, p5, p6, p7, p8;
  int board_size = 25;
  int half_rect_width = 0.015 / RESOLUTION + 0.5;
  int half_rect_height = 0.015 / RESOLUTION + 0.5;

  // outer rectangle, p1~p4 all rectangle certers
  p1.x = board_size + half_rect_width;
  p1.y = board_size + half_rect_height;

  p2.x = p1.x;
  p2.y = p1.y + 2 * half_rect_height + (int)(0.04 / RESOLUTION + 0.5);

  p3.x = p1.x;
  p3.y = p2.y + 2 * half_rect_height + (int)(1.03 / RESOLUTION + 0.5);

  p4.x = p1.x;
  p4.y = p3.y + 2 * half_rect_height + (int)(0.04 / RESOLUTION + 0.5);

  p5.x = p1.x + 2 * half_rect_width + 4 * half_rect_width +
         (int)((0.54 + 2 * 0.04) / RESOLUTION + 0.5);
  p5.y = p1.y;

  p6.x = p2.x + 2 * half_rect_width + 4 * half_rect_width +
         (int)((0.54 + 2 * 0.04) / RESOLUTION + 0.5);
  p6.y = p2.y;

  p7.x = p3.x + 2 * half_rect_width + 4 * half_rect_width +
         (int)((0.54 + 2 * 0.04) / RESOLUTION + 0.5);
  p7.y = p3.y;

  p8.x = p4.x + 2 * half_rect_width + 4 * half_rect_width +
         (int)((0.54 + 2 * 0.04) / RESOLUTION + 0.5);
  p8.y = p4.y;

  Point p1_1, p1_2, p4_1, p4_2;
  p1_1.x = p1.x + 2 * half_rect_width + (int)(0.04 / RESOLUTION + 0.5);
  p1_1.y = p1.y;

  p1_2.x = p1.x + 4 * half_rect_width + (int)((0.04 + 0.54) / RESOLUTION + 0.5);
  p1_2.y = p1.y;

  p4_1.x = p4.x + 2 * half_rect_width + (int)((0.04) / RESOLUTION + 0.5);
  p4_1.y = p4.y;

  p4_2.x = p4.x + 4 * half_rect_width + (int)((0.04 + 0.54) / RESOLUTION + 0.5);
  p4_2.y = p4.y;

  int width = p8.x + board_size + half_rect_width;
  int height = p8.y + board_size + +half_rect_height;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
           cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Point(p1.x + half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
           cv::Point(p2.x + half_rect_width, p2.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
           cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
           cv::Point(p3.x + half_rect_width, p3.y - half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x - half_rect_width, p4.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x + half_rect_width, p4.y - half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p5.x - half_rect_width, p5.y - half_rect_height),
           cv::Point(p5.x - half_rect_width, p5.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p5.x - half_rect_width, p5.y - half_rect_height),
  // cv::Point(p5.x +
  // half_rect_width, p5.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p5.x - half_rect_width, p5.y + half_rect_height),
           cv::Point(p5.x + half_rect_width, p5.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p6.x - half_rect_width, p6.y - half_rect_height),
           cv::Point(p6.x - half_rect_width, p6.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p5.x - half_rect_width, p5.y - half_rect_height),
  // cv::Point(p5.x +
  // half_rect_width, p5.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p6.x - half_rect_width, p6.y + half_rect_height),
           cv::Point(p6.x + half_rect_width, p6.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p7.x - half_rect_width, p7.y - half_rect_height),
           cv::Point(p7.x - half_rect_width, p7.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p7.x - half_rect_width, p7.y - half_rect_height),
  // cv::Point(p7.x +
  // half_rect_width, p7.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p7.x - half_rect_width, p7.y - half_rect_height),
           cv::Point(p7.x + half_rect_width, p7.y - half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p8.x - half_rect_width, p8.y - half_rect_height),
           cv::Point(p8.x - half_rect_width, p8.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p7.x - half_rect_width, p7.y - half_rect_height),
  // cv::Point(p7.x +
  // half_rect_width, p7.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p8.x - half_rect_width, p8.y - half_rect_height),
           cv::Point(p8.x + half_rect_width, p8.y - half_rect_height),
           cv::Scalar(255));

  cv::line(model,
           cv::Point(p1_1.x - half_rect_width, p1_1.y - half_rect_height),
           cv::Point(p1_1.x - half_rect_width, p1_1.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p9.x - half_rect_width, p9.y - half_rect_height),
  // cv::Point(p9.x +
  // half_rect_width, p9.y - half_rect_height), cv::Scalar(255));
  cv::line(model,
           cv::Point(p1_1.x - half_rect_width, p1_1.y + half_rect_height),
           cv::Point(p1_1.x + half_rect_width, p1_1.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model,
           cv::Point(p1_2.x - half_rect_width, p1_2.y - half_rect_height),
           cv::Point(p1_2.x - half_rect_width, p1_2.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p11.x - half_rect_width, p11.y -
  // half_rect_height), cv::Point(p11.x +
  // half_rect_width, p11.y - half_rect_height), cv::Scalar(255));
  cv::line(model,
           cv::Point(p1_2.x - half_rect_width, p1_2.y + half_rect_height),
           cv::Point(p1_2.x + half_rect_width, p1_2.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model,
           cv::Point(p4_1.x - half_rect_width, p4_1.y - half_rect_height),
           cv::Point(p4_1.x - half_rect_width, p4_1.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p13.x - half_rect_width, p13.y -
  // half_rect_height), cv::Point(p13.x +
  // half_rect_width, p13.y - half_rect_height), cv::Scalar(255));
  cv::line(model,
           cv::Point(p4_1.x - half_rect_width, p4_1.y - half_rect_height),
           cv::Point(p4_1.x + half_rect_width, p4_1.y - half_rect_height),
           cv::Scalar(255));

  cv::line(model,
           cv::Point(p4_2.x - half_rect_width, p4_2.y - half_rect_height),
           cv::Point(p4_2.x - half_rect_width, p4_2.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p13.x - half_rect_width, p13.y -
  // half_rect_height), cv::Point(p13.x +
  // half_rect_width, p13.y - half_rect_height), cv::Scalar(255));
  cv::line(model,
           cv::Point(p4_2.x - half_rect_width, p4_2.y - half_rect_height),
           cv::Point(p4_2.x + half_rect_width, p4_2.y - half_rect_height),
           cv::Scalar(255));

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);
  return model.clone();
}

// 20190619
// 天信货架，圆形脚，之后所有圆形货架都以这个为模板
Mat template_shelf_circle_model13_tx() {
  Point p1, p2, p3, p4;
  int board_size = 20;

  int radius = 0.0125 / RESOLUTION + 0.5;

  p1.x = board_size + radius;
  p1.y = board_size + radius;
  p2.x = p1.x;
  p2.y = p1.y + (int)(1.282 / RESOLUTION + 0.5);

  p3.x = p1.x + (int)(0.612 / RESOLUTION + 0.5);
  p3.y = p1.y;
  p4.x = p2.x + (int)(0.612 / RESOLUTION + 0.5);
  p4.y = p2.y;

  int width = p4.x + board_size;
  int height = p4.y + board_size;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  circle(model, p1, radius, Scalar(255), 2);
  circle(model, p2, radius, Scalar(255), 2);
  circle(model, p3, radius, Scalar(255), 2);
  circle(model, p4, radius, Scalar(255), 2);

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);

  return model.clone();
}
// 20190624
// 深圳共进
Mat template_shelf_circle_model14_szgj() {
  Point p1, p2, p3, p4;
  int board_size = 20;

  int radius = 0.015 / RESOLUTION + 0.5;

  p1.x = board_size + radius;
  p1.y = board_size + radius;
  p2.x = p1.x;
  p2.y = p1.y + (int)(1.28 / RESOLUTION + 0.5);

  p3.x = p1.x + (int)(0.62 / RESOLUTION + 0.5);
  p3.y = p1.y;
  p4.x = p2.x + (int)(0.62 / RESOLUTION + 0.5);
  p4.y = p2.y;

  int width = p4.x + board_size;
  int height = p4.y + board_size;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  circle(model, p1, radius, Scalar(255), 2);
  circle(model, p2, radius, Scalar(255), 2);
  circle(model, p3, radius, Scalar(255), 2);
  circle(model, p4, radius, Scalar(255), 2);

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);

  return model.clone();
}

// 20190626
// 爱立信，矩形货架，四个角
Mat template_shelf_square_model_alx() {
  Point p1, p2, p3, p4, p5, p6, p7, p8;
  int board_size = 25;
  int half_rect_width = 0.04 / RESOLUTION + 0.5;
  int half_rect_height = 0.04 / RESOLUTION + 0.5;

  // outer rectangle, p1~p4 all rectangle certers
  p1.x = board_size + half_rect_width;
  p1.y = board_size + half_rect_height;

  p2.x = p1.x;
  p2.y = p1.y + 2 * half_rect_height + (int)(1.03 / RESOLUTION + 0.5);

  p3.x = p1.x + 2 * half_rect_width + (int)(0.65 / RESOLUTION + 0.5);
  p3.y = p1.y;

  p4.x = p2.x + 2 * half_rect_width + (int)(0.65 / RESOLUTION + 0.5);
  p4.y = p2.y;

  int width = p4.x + board_size + half_rect_width;
  int height = p4.y + board_size + +half_rect_height;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
           cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Point(p1.x + half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x + half_rect_width, p2.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
           cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Point(p3.x + half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x + half_rect_width, p4.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x - half_rect_width, p4.y + half_rect_height),
           cv::Scalar(255));

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);
  return model.clone();
}

// 20190701
// 天信货架
Mat template_shelf_circle_model_tx2() {
  Point p1, p2, p3, p4;
  int board_size = 20;

  int radius = 0.015 / RESOLUTION + 0.5;

  p1.x = board_size + radius;
  p1.y = board_size + radius;
  p2.x = p1.x;
  p2.y = p1.y + (int)(1.28 / RESOLUTION + 0.5);

  p3.x = p1.x + (int)(0.60 / RESOLUTION + 0.5);
  p3.y = p1.y;
  p4.x = p2.x + (int)(0.60 / RESOLUTION + 0.5);
  p4.y = p2.y;

  int width = p4.x + board_size;
  int height = p4.y + board_size;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  circle(model, p1, radius, Scalar(255), 2);
  circle(model, p2, radius, Scalar(255), 2);
  circle(model, p3, radius, Scalar(255), 2);
  circle(model, p4, radius, Scalar(255), 2);

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);

  return model.clone();
}

Mat template_chargingPile() {
  Point p1, p2, p3;
  int board_size = 25;
  double angle = 120. / 2 * CV_PI / 180;
  p1.x = board_size;
  p1.y = board_size;
  p2.x = board_size + (int)(0.2 * cos(angle) / RESOLUTION + 0.5);
  p2.y = board_size + (int)(0.2 * sin(angle) / RESOLUTION + 0.5);
  p3.x = board_size;
  p3.y = p2.y + (int)(0.2 * sin(angle) / RESOLUTION + 0.5);

  int width = p2.x + board_size;
  int height = p3.y + board_size;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  line(model, p1, p2, Scalar(255));
  line(model, p2, p3, Scalar(255));
  GaussianBlur(model, model, Size(0, 0), 5);
  model = model * 5;
  model = model * 255 / 90;

  return model.clone();
}

Mat template_chargingPile2() {
  Point p1, p2, p3, p4;
  int board_size = 25;
  double angle = 100. / 2 * CV_PI / 180;
  p1.x = board_size;
  p1.y = board_size;
  p2.x = board_size + (int)(0.2 * cos(angle) / RESOLUTION + 0.5);
  p2.y = board_size + (int)(0.2 * sin(angle) / RESOLUTION + 0.5);
  p3.x = p2.x;
  p3.y = p2.y + (int)(0.25 / RESOLUTION + 0.5);
  p4.x = board_size;
  p4.y = p3.y + (int)(0.2 * sin(angle) / RESOLUTION + 0.5);

  int width = p3.x + board_size;
  int height = p4.y + board_size;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  line(model, p1, p2, Scalar(255));
  line(model, p2, p3, Scalar(255));
  line(model, p3, p4, Scalar(255));
  GaussianBlur(model, model, Size(0, 0), 5);
  model = model * 5;
  model = model * 255 / 80;

  return model.clone();
}

Mat template_chargingPile3() {
  Point p1, p2, p3, p4;
  int board_size = 25;
  double angle = 90. / 2 * CV_PI / 180;
  p1.x = board_size;
  p1.y = board_size;
  p2.x = board_size + (int)(0.2 * cos(angle) / RESOLUTION + 0.5);
  p2.y = board_size + (int)(0.2 * sin(angle) / RESOLUTION + 0.5);
  p3.x = board_size;
  p3.y = p2.y + (int)(0.2 * sin(angle) / RESOLUTION + 0.5);
  p4.x = board_size;
  p4.y = p3.y + (int)(0.2 / RESOLUTION + 0.5);

  int width = p2.x + board_size;
  int height = p4.y + board_size;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  line(model, p1, p2, Scalar(255));
  line(model, p2, p3, Scalar(255));
  line(model, p3, p4, Scalar(255));
  GaussianBlur(model, model, Size(0, 0), 5);
  model = model * 5;
  model = model * 255 / 70;

  return model.clone();
}

Mat template_chargingPile4() {
  Point p1, p2, p3, p4, p5, p6;
  int board_size = 15;
  double cos_a = 145. / 200;
  double sin_a = sqrt(1 - cos_a * cos_a);
  p1.x = board_size;
  p1.y = board_size;
  p2.x = board_size;
  p2.y = board_size + int(0.2 / RESOLUTION + 0.5);
  p3.x = board_size + int(0.145 / RESOLUTION + 0.5);
  p3.y = p2.y + int(0.2 * sin_a / RESOLUTION + 0.5);
  p4.x = p3.x;
  p4.y = p3.y + int((0.285 - 2 * 0.2 * sin_a) / RESOLUTION + 0.5);
  p5.x = board_size;
  p5.y = board_size + int(0.485 / RESOLUTION + 0.5);
  p6.x = board_size;
  p6.y = board_size + int(0.6 / RESOLUTION + 0.5);

  int width = p3.x + board_size;
  int height = p6.y + board_size;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  line(model, p1, p2, Scalar(255));
  line(model, p2, p3, Scalar(255));
  line(model, p3, p4, Scalar(255));
  line(model, p4, p5, Scalar(255));
  line(model, p5, p6, Scalar(255));
  GaussianBlur(model, model, Size(0, 0), 5);
  model = model * 5;
  model = model * 255 / 80;

  return model.clone();
}

cv::Mat template_chargingPile500_2() {
  int board_size = 25;
  cv::Point2d p1, p2, p3, p4, p5, p6, p7, p8, p9;
  p1.x = board_size;
  p1.y = board_size;

  p2.x = board_size + 0.07 / RESOLUTION + 0.5;
  p2.y = board_size;

  p3.x = board_size + 0.21325 / RESOLUTION + 0.5;
  p3.y = board_size + 0.14 / RESOLUTION + 0.5;

  p4.x = board_size + 0.3565 / RESOLUTION + 0.5;
  p4.y = board_size;

  p5.x = board_size + 0.5475 / RESOLUTION + 0.5;
  p5.y = board_size;

  p6.x = board_size + 0.69075 / RESOLUTION + 0.5;
  p6.y = board_size + 0.14 / RESOLUTION + 0.5;

  p7.x = board_size + 0.834 / RESOLUTION + 0.5;
  p7.y = board_size;

  p8.x = board_size + 0.904 / RESOLUTION + 0.5;
  p8.y = board_size;

  cv::Mat model(board_size + p3.y, board_size + p8.x, CV_8UC1, cv::Scalar(0));

  cv::line(model, p1, p2, cv::Scalar(255), 1);
  cv::line(model, p2, p3, cv::Scalar(255), 1);
  cv::line(model, p3, p4, cv::Scalar(255), 1);
  cv::line(model, p4, p5, cv::Scalar(255), 1);
  cv::line(model, p5, p6, cv::Scalar(255), 1);
  cv::line(model, p6, p7, cv::Scalar(255), 1);
  cv::line(model, p7, p8, cv::Scalar(255), 1);

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);

  return model;
}
Mat template_chargingPile_KM() {
  Point p1, p2, p3, p4, p5;
  int board_size = 25;
  p1.x = board_size;
  p1.y = board_size;
  p2.x = p1.x;
  p2.y = p1.y + int(0.0875 / RESOLUTION + 0.5);

  p3.x = p2.x + int(0.14 / RESOLUTION + 0.5);
  p3.y = p2.y + int(0.14 / RESOLUTION + 0.5);

  p4.x = p1.x;
  p4.y = p3.y + int(0.14 / RESOLUTION + 0.5);

  p5.x = p4.x;
  p5.y = p4.y + int(0.0875 / RESOLUTION + 0.5);

  int width = p3.x + board_size;
  int height = p5.y + board_size;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  line(model, p1, p2, Scalar(255));
  line(model, p2, p3, Scalar(255));
  line(model, p3, p4, Scalar(255));
  line(model, p4, p5, Scalar(255));
  GaussianBlur(model, model, Size(0, 0), 5);
  model = model * 5;
  model = model * 255 / 80;

  return model.clone();
}

void template_generate_test(int argc, char** argv) {
  ros::init(argc, argv, "read_image_to_ros");
  ros::start();

  ros::NodeHandle nh("~");

  ros::ServiceClient template_genertor =
      nh.serviceClient<visual_servo_msgs::TemplateGenerator>(
          "/template_generator_server");
  LOG(INFO) << "AAAAAA";
  //* ft_01
  {
  //   // init point_array

  //   visual_servo_msgs::Point point[4];
  //   visual_servo_msgs::PointArray point_array;
  //   // init line_array
  //   visual_servo_msgs::Line line;
  //   visual_servo_msgs::LineArray line_array;
  //   point[0].r = point[1].r = point[2].r = point[3].r = 0.04;
  //   point[0].x = point[3].x = 0;
  //   point[1].x = point[2].x = 0.75;
  //   point[0].y = point[1].y = 0;
  //   point[3].y = point[2].y = 1.03;
  //   point_array.points.push_back(point[0]);
  //   point_array.points.push_back(point[1]);
  //   point_array.points.push_back(point[2]);
  //   point_array.points.push_back(point[3]);

  //   line.points[0].r = 0.002;
  //   line.points[1].r = 0.002;

  //   // 1
  //   line.points[0].x = -0.08;
  //   line.points[0].y = 0.08;
  //   line.points[1].x = -0.08;
  //   line.points[1].y = -0.08;
  //   line_array.lines.push_back(line);
  //   // 2
  //   line.points[0].x = 0.08;
  //   line.points[0].y = 0.08;
  //   line.points[1].x = -0.08;
  //   line.points[1].y = 0.08;
  //   line_array.lines.push_back(line);
  //   // 3
  //   line.points[0].x = 0.67;
  //   line.points[0].y = 1.11;
  //   line.points[1].x = 0.67;
  //   line.points[1].y = 0.95;
  //   line_array.lines.push_back(line);
  //   // 4
  //   line.points[0].x = 0.83;
  //   line.points[0].y = 0.95;
  //   line.points[1].x = 0.67;
  //   line.points[1].y = 0.95;
  //   line_array.lines.push_back(line);
  //   // 5
  //   line.points[0].x = 0.67;
  //   line.points[0].y = 0.08;
  //   line.points[1].x = 0.67;
  //   line.points[1].y = -0.08;
  //   line_array.lines.push_back(line);
  //   // 6
  //   line.points[0].x = 0.83;
  //   line.points[0].y = 0.08;
  //   line.points[1].x = 0.67;
  //   line.points[1].y = 0.08;
  //   line_array.lines.push_back(line);
  //   // 7
  //   line.points[0].x = -0.08;
  //   line.points[0].y = 1.11;
  //   line.points[1].x = -0.08;
  //   line.points[1].y = 0.95;
  //   line_array.lines.push_back(line);
  //   // 8
  //   line.points[0].x = 0.08;
  //   line.points[0].y = 0.95;
  //   line.points[1].x = -0.08;
  //   line.points[1].y = 0.95;
  //   line_array.lines.push_back(line);

  //   visual_servo_msgs::TemplateGenerator template_genertor_query;
  //   template_genertor_query.request.point_array = point_array;
  //   template_genertor_query.request.line_array = line_array;
  //   template_genertor_query.request.templete_path =
  //       "/home/lile/ft/ft_01_algo.png";

  //   if (template_genertor.call(template_genertor_query)) {
  //     if (template_genertor_query.response.error_message == "") {
  //     }
  //   }
  }
  /**/

  //* ft_02
  {
    // init point_array

    visual_servo_msgs::Point point[4];
    visual_servo_msgs::PointArray point_array;
    // init line_array
    visual_servo_msgs::Line line;
    visual_servo_msgs::LineArray line_array;
    point[0].r = point[1].r = point[2].r = point[3].r = 0.04;
    point[0].x = point[3].x = 0;
    point[1].x = point[2].x = 1.65;
    point[0].y = point[1].y = 0;
    point[3].y = point[2].y = 1.05;
    point_array.points.push_back(point[0]);
    point_array.points.push_back(point[1]);
    point_array.points.push_back(point[2]);
    point_array.points.push_back(point[3]);

    line.points[0].r = 0.002;
    line.points[1].r = 0.002;

    line.points[0].x = -0.075;
    line.points[0].y = 0.075;
    line.points[1].x = -0.075;
    line.points[1].y = -0.075;
    line_array.lines.push_back(line);

    line.points[0].x = 0.075;
    line.points[0].y = 0.075;
    line.points[1].x = -0.075;
    line.points[1].y = 0.075;
    line_array.lines.push_back(line);

    line.points[0].x = 1.575;
    line.points[0].y = 1.125;
    line.points[1].x = 1.575;
    line.points[1].y = 0.975;
    line_array.lines.push_back(line);

    line.points[0].x = 1.725;
    line.points[0].y = 0.975;
    line.points[1].x = 1.575;
    line.points[1].y = 0.975;
    line_array.lines.push_back(line);

    line.points[0].x = 1.575;
    line.points[0].y = 0.075;
    line.points[1].x = 1.575;
    line.points[1].y = -0.075;
    line_array.lines.push_back(line);

    line.points[0].x = 1.725;
    line.points[0].y = 0.075;
    line.points[1].x = 1.575;
    line.points[1].y = 0.075;
    line_array.lines.push_back(line);

    line.points[0].x = -0.075;
    line.points[0].y = 1.125;
    line.points[1].x = -0.075;
    line.points[1].y = 0.975;
    line_array.lines.push_back(line);

    line.points[0].x = 0.075;
    line.points[0].y = 0.975;
    line.points[1].x = -0.075;
    line.points[1].y = 0.975;
    line_array.lines.push_back(line);

    visual_servo_msgs::TemplateGenerator template_genertor_query;
    template_genertor_query.request.point_array = point_array;
    template_genertor_query.request.line_array = line_array;
    template_genertor_query.request.templete_path =
        "/home/juling/ft_02_algo.png";
    LOG(INFO) << "BBBB";
    if (template_genertor.call(template_genertor_query)) {
      if (template_genertor_query.response.error_message == "") {
      }
    }
  }
  /**/

  //* ft_03
  {
  //   // init point_array

  //   visual_servo_msgs::Point point[4];
  //   visual_servo_msgs::PointArray point_array;
  //   // init line_array
  //   visual_servo_msgs::Line line;
  //   visual_servo_msgs::LineArray line_array;
  //   point[0].r = point[1].r = point[2].r = point[3].r = 0.04;
  //   point[0].x = point[3].x = 0;
  //   point[1].x = point[2].x = 1.25;
  //   point[0].y = point[1].y = 0;
  //   point[3].y = point[2].y = 1.02;
  //   point_array.points.push_back(point[0]);
  //   point_array.points.push_back(point[1]);
  //   point_array.points.push_back(point[2]);
  //   point_array.points.push_back(point[3]);

  //   line.points[0].r = 0.002;
  //   line.points[1].r = 0.002;

  //   line.points[0].x = -0.075;
  //   line.points[0].y = 0.075;
  //   line.points[1].x = -0.075;
  //   line.points[1].y = -0.075;
  //   line_array.lines.push_back(line);
  //   // 2
  //   line.points[0].x = 0.075;
  //   line.points[0].y = 0.075;
  //   line.points[1].x = -0.075;
  //   line.points[1].y = 0.075;
  //   line_array.lines.push_back(line);
  //   // 3
  //   line.points[0].x = 1.175;
  //   line.points[0].y = 1.095;
  //   line.points[1].x = 1.175;
  //   line.points[1].y = 0.945;
  //   line_array.lines.push_back(line);
  //   // 4
  //   line.points[0].x = 1.325;
  //   line.points[0].y = 0.945;
  //   line.points[1].x = 1.175;
  //   line.points[1].y = 0.945;
  //   line_array.lines.push_back(line);
  //   // 5
  //   line.points[0].x = 1.175;
  //   line.points[0].y = 0.075;
  //   line.points[1].x = 1.175;
  //   line.points[1].y = -0.075;
  //   line_array.lines.push_back(line);
  //   // 6
  //   line.points[0].x = 1.325;
  //   line.points[0].y = 0.075;
  //   line.points[1].x = 1.175;
  //   line.points[1].y = 0.075;
  //   line_array.lines.push_back(line);
  //   // 7
  //   line.points[0].x = -0.075;
  //   line.points[0].y = 1.095;
  //   line.points[1].x = -0.075;
  //   line.points[1].y = 0.945;
  //   line_array.lines.push_back(line);
  //   // 8
  //   line.points[0].x = 0.075;
  //   line.points[0].y = 0.945;
  //   line.points[1].x = -0.075;
  //   line.points[1].y = 0.945;
  //   line_array.lines.push_back(line);

  //   visual_servo_msgs::TemplateGenerator template_genertor_query;
  //   template_genertor_query.request.point_array = point_array;
  //   template_genertor_query.request.line_array = line_array;
  //   template_genertor_query.request.templete_path =
  //       "/home/lile/ft/载货4_algo.png";

  //   if (template_genertor.call(template_genertor_query)) {
  //     if (template_genertor_query.response.error_message == "") {
  //     }
  //   }
  }
  /**/
}

// 20190718
// 碧桂园，矩形货架，四个角
Mat template_shelf_square_model_bgy() {
  Point p1, p2, p3, p4;
  int board_size = 25;
  int half_rect_width = 0.025 / RESOLUTION + 0.5;
  int half_rect_height = 0.025 / RESOLUTION + 0.5;

  // outer rectangle, p1~p4 all rectangle certers
  p1.x = board_size + half_rect_width;
  p1.y = board_size + half_rect_height;

  p2.x = p1.x;
  p2.y = p1.y + (int)(1.20 / RESOLUTION + 0.5);

  p3.x = p1.x + (int)(1.20 / RESOLUTION + 0.5);
  p3.y = p1.y;

  p4.x = p2.x + (int)(1.20 / RESOLUTION + 0.5);
  p4.y = p2.y;

  int width = p4.x + board_size + half_rect_width;
  int height = p4.y + board_size + +half_rect_height;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
           cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Point(p1.x + half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x + half_rect_width, p2.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
           cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Point(p3.x + half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x + half_rect_width, p4.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x - half_rect_width, p4.y + half_rect_height),
           cv::Scalar(255));

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);
  return model.clone();
}

// 20190812
// 惠州，矩形货架，两个角
Mat template_shelf_square_model_hz1() {
  Point p1, p2;
  int board_size = 25;
  int half_rect_width = 0.055 / RESOLUTION + 0.5;
  int half_rect_height = 0.034 / RESOLUTION + 0.5;

  // outer rectangle, p1~p4 all rectangle certers
  p1.x = board_size + half_rect_width;
  p1.y = board_size + half_rect_height;

  p2.x = p1.x;
  p2.y = p1.y + (int)(1.082 / RESOLUTION + 0.5);

  int width = p2.x + board_size + half_rect_width;
  int height = p2.y + board_size + +half_rect_height;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
           cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Point(p1.x + half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x + half_rect_width, p2.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
           cv::Scalar(255));

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);
  return model.clone();
}

// 20190812
// 惠州，矩形货架，四个角
Mat template_shelf_square_model_hz2() {
  Point p1, p2, p3, p4;
  int board_size = 25;
  int half_rect_width = 0.0325 / RESOLUTION + 0.5;
  int half_rect_height = 0.029 / RESOLUTION + 0.5;

  // outer rectangle, p1~p4 all rectangle certers
  p1.x = board_size + half_rect_width;
  p1.y = board_size + half_rect_height;

  p2.x = p1.x;
  p2.y = p1.y + 2 * half_rect_height + (int)(1.1185 / RESOLUTION + 0.5);

  p3.x = p1.x + 2 * half_rect_width + (int)(1.362 / RESOLUTION + 0.5);
  p3.y = p1.y;

  p4.x = p2.x + 2 * half_rect_width + (int)(1.362 / RESOLUTION + 0.5);
  p4.y = p2.y;

  int width = p4.x + board_size + half_rect_width;
  int height = p4.y + board_size + +half_rect_height;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
           cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Point(p1.x + half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x + half_rect_width, p2.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
           cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Point(p3.x + half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x + half_rect_width, p4.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x - half_rect_width, p4.y + half_rect_height),
           cv::Scalar(255));

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);
  return model.clone();
}

// 20190906
// 碧桂园，矩形货架，四个角
Mat template_shelf_square_model_bgy20190906() {
  Point p1, p2, p3, p4;

  float L1, L2, L3, L4;
  L1 = 1.13;
  L2 = 0.595;
  L3 = 0.045;
  L4 = 0.04;

  int board_size = 25;
  int half_rect_width = (L4 / 2) / RESOLUTION + 0.5;
  int half_rect_height = (L3 / 2) / RESOLUTION + 0.5;

  // outer rectangle, p1~p4 all rectangle certers
  p1.x = board_size + half_rect_width;
  p1.y = board_size + half_rect_height;

  p2.x = p1.x;
  p2.y = p1.y + 2 * half_rect_height + (int)(L1 / RESOLUTION + 0.5);

  p3.x = p1.x + 2 * half_rect_width + (int)(L2 / RESOLUTION + 0.5);
  p3.y = p1.y;

  p4.x = p2.x + 2 * half_rect_width + (int)(L2 / RESOLUTION + 0.5);
  p4.y = p2.y;

  int width = p4.x + board_size + half_rect_width;
  int height = p4.y + board_size + +half_rect_height;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
           cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Point(p1.x + half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x + half_rect_width, p2.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
           cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Point(p3.x + half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x + half_rect_width, p4.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x - half_rect_width, p4.y + half_rect_height),
           cv::Scalar(255));

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);
  return model.clone();
}
// 20191014
// 宜兴中环半导体
Mat template_shelf_square_model_yxzh() {
  Point p1, p2, p3, p5, p6, p7;
  int board_size = 25;
  int half_rect_width = (0.0375 / 2) / RESOLUTION + 0.5;
  int half_rect_height = (0.0375 / 2) / RESOLUTION + 0.5;

  // outer rectangle, p1~p4 all rectangle certers
  p1.x = board_size + half_rect_width;
  p1.y = board_size + half_rect_height;

  p2.x = p1.x;
  p2.y = p1.y + 2 * half_rect_height + (int)(0.8 / RESOLUTION + 0.5);

  p3.x = p1.x;
  p3.y = p2.y + 2 * half_rect_height + (int)(0.8 / RESOLUTION + 0.5);

  p5.x = p1.x + 2 * half_rect_width + (int)((0.582) / RESOLUTION + 0.5);
  p5.y = p1.y;

  p6.x = p5.x;
  p6.y = p2.y;

  p7.x = p5.x;
  p7.y = p3.y;

  int width = p7.x + board_size + half_rect_width;
  int height = p7.y + board_size + +half_rect_height;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
           cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Point(p1.x + half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
           cv::Point(p2.x + half_rect_width, p2.y + half_rect_height),
           cv::Scalar(255));
  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x + half_rect_width, p2.y - half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
           cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
           cv::Point(p3.x + half_rect_width, p3.y - half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p5.x - half_rect_width, p5.y - half_rect_height),
           cv::Point(p5.x - half_rect_width, p5.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p5.x - half_rect_width, p5.y - half_rect_height),
  // cv::Point(p5.x +
  // half_rect_width, p5.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p5.x - half_rect_width, p5.y + half_rect_height),
           cv::Point(p5.x + half_rect_width, p5.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p6.x - half_rect_width, p6.y - half_rect_height),
           cv::Point(p6.x - half_rect_width, p6.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p5.x - half_rect_width, p5.y - half_rect_height),
  // cv::Point(p5.x +
  // half_rect_width, p5.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p6.x - half_rect_width, p6.y + half_rect_height),
           cv::Point(p6.x + half_rect_width, p6.y + half_rect_height),
           cv::Scalar(255));
  cv::line(model, cv::Point(p6.x - half_rect_width, p6.y - half_rect_height),
           cv::Point(p6.x + half_rect_width, p6.y - half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p7.x - half_rect_width, p7.y - half_rect_height),
           cv::Point(p7.x - half_rect_width, p7.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p7.x - half_rect_width, p7.y - half_rect_height),
  // cv::Point(p7.x +
  // half_rect_width, p7.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p7.x - half_rect_width, p7.y - half_rect_height),
           cv::Point(p7.x + half_rect_width, p7.y - half_rect_height),
           cv::Scalar(255));

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);
  return model.clone();
}

// 20191205 中兴老化柜
void zte() {
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(0);

  int board_size = 25;
  // 4层老化柜
  {
    float w = 0.774, h = 1.438;
    cv::Point p1, p2, p3, p4;
    p1.x = board_size;
    p1.y = board_size;

    p2.x = p1.x;
    p2.y = p1.y + h / RESOLUTION + 0.5;

    p3.x = p1.x + w / RESOLUTION + 0.5;
    p3.y = p1.y;

    p4.x = p3.x;
    p4.y = p2.y;

    int width = p4.x + board_size;
    int height = p4.y + board_size;
    Mat model = Mat::zeros(height, width, CV_8UC1);

    cv::line(model, p1, p2, cv::Scalar(255));
    cv::line(model, p1, p3, cv::Scalar(255));
    cv::line(model, p2, p4, cv::Scalar(255));
    cv::line(model, p3, p4, cv::Scalar(255));

    GaussianBlur(model, model, Size(0, 0), 5);
    cv::normalize(model, model, 0, 255, NORM_MINMAX, CV_8UC1);

    std::string image_path = getenv("HOME") + std::string("/layer4.png");
    imwrite(image_path, model, compression_params);
  }

  // 3层老化柜
  {
    float w = 0.824, h = 1.504;
    float h_offset = 0.1, w_offset = 0.121;
    cv::Point p1, p2, p3, p4, p5, p6, p7, p8;

    p6.x = w_offset / RESOLUTION + board_size;
    p6.y = board_size;

    p5.x = p6.x;
    p5.y = p6.y + h_offset / RESOLUTION;

    p7.x = p6.x + 0.604 / RESOLUTION;
    p7.y = p6.y;

    p8.x = p7.x;
    p8.y = p5.y;

    p1.x = board_size;
    p1.y = p5.y;

    p2.x = p1.x;
    p2.y = p1.y + h / RESOLUTION + 0.5;

    p3.x = p1.x + w / RESOLUTION + 0.5;
    p3.y = p1.y;

    p4.x = p3.x;
    p4.y = p2.y;

    int width = p4.x + board_size;
    int height = p4.y + board_size;
    Mat model = Mat::zeros(height, width, CV_8UC1);

    cv::line(model, p1, p5, cv::Scalar(255));
    cv::line(model, p1, p2, cv::Scalar(255));
    cv::line(model, p5, p6, cv::Scalar(255));
    cv::line(model, p6, p7, cv::Scalar(255));
    cv::line(model, p7, p8, cv::Scalar(255));
    cv::line(model, p8, p3, cv::Scalar(255));
    cv::line(model, p3, p4, cv::Scalar(255));
    cv::line(model, p2, p4, cv::Scalar(255));

    GaussianBlur(model, model, Size(0, 0), 5);
    cv::normalize(model, model, 0, 255, NORM_MINMAX, CV_8UC1);

    std::string image_path = getenv("HOME") + std::string("/layer3.png");
    imwrite(image_path, model, compression_params);
  }

  // 2层老化柜
  {
    float w = 0.826, h = 1.616;
    cv::Point p1, p2, p3, p4;
    p1.x = board_size;
    p1.y = board_size;

    p2.x = p1.x;
    p2.y = p1.y + h / RESOLUTION + 0.5;

    p3.x = p1.x + w / RESOLUTION + 0.5;
    p3.y = p1.y;

    p4.x = p3.x;
    p4.y = p2.y;

    int width = p4.x + board_size;
    int height = p4.y + board_size;
    Mat model = Mat::zeros(height, width, CV_8UC1);

    cv::line(model, p1, p2, cv::Scalar(255));
    cv::line(model, p1, p3, cv::Scalar(255));
    cv::line(model, p2, p4, cv::Scalar(255));
    cv::line(model, p3, p4, cv::Scalar(255));

    GaussianBlur(model, model, Size(0, 0), 5);
    cv::normalize(model, model, 0, 255, NORM_MINMAX, CV_8UC1);

    std::string image_path = getenv("HOME") + std::string("/layer2.png");
    imwrite(image_path, model, compression_params);
  }
}

// 20200403 栈板测试
void zhanban_test() {
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(0);

  int board_size = 25;
  cv::Point p1, p2, p3, p4, p5, p6;
  cv::Point p11, p22, p33, p44, p55, p66;
  p1.x = board_size;
  p1.y = board_size;

  p2.x = p1.x;
  p3.x = p1.x;
  p4.x = p1.x;
  p5.x = p1.x;
  p6.x = p1.x;

  p2.y = p1.y + 0.087 / RESOLUTION + 0.5;
  p3.y = p1.y + (0.087 + 0.365) / RESOLUTION + 0.5;
  p4.y = p1.y + (0.087 + 0.365 + 0.087) / RESOLUTION + 0.5;
  p5.y = p1.y + (0.087 + 0.365 + 0.087 + 0.37) / RESOLUTION + 0.5;
  p6.y = p1.y + (0.087 + 0.365 + 0.087 + 0.37 + 0.087) / RESOLUTION + 0.5;

  p11.x = p1.x + 0.0865 / RESOLUTION + 0.5;
  p22.x = p11.x;
  p33.x = p11.x;
  p44.x = p11.x;
  p55.x = p11.x;
  p66.x = p11.x;

  p11.y = p1.y;
  p22.y = p2.y;
  p33.y = p3.y;
  p44.y = p4.y;
  p55.y = p5.y;
  p66.y = p6.y;

  int width = p66.x + board_size;
  int height = p66.y + board_size;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  cv::line(model, p1, p2, cv::Scalar(255));
  cv::line(model, p3, p4, cv::Scalar(255));
  cv::line(model, p5, p6, cv::Scalar(255));
  // cv::line(model, p1, p11, cv::Scalar(255));
  cv::line(model, p2, p22, cv::Scalar(255));
  cv::line(model, p3, p33, cv::Scalar(255));
  cv::line(model, p4, p44, cv::Scalar(255));
  cv::line(model, p5, p55, cv::Scalar(255));
  // cv::line(model, p6, p66, cv::Scalar(255));

  GaussianBlur(model, model, Size(0, 0), 5);
  cv::normalize(model, model, 0, 255, NORM_MINMAX, CV_8UC1);

  std::string image_path = getenv("HOME") + std::string("/zhanban.png");
  imwrite(image_path, model, compression_params);
}

void zte_test() {
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(0);

  int board_size = 25;
  // test1
  {
    float w = 4.5, h = 0.5;
    cv::Point p1, p2, p3, p4;
    p1.x = board_size;
    p1.y = board_size;

    p2.x = p1.x;
    p2.y = p1.y + h / RESOLUTION + 0.5;

    p3.x = p1.x + w / RESOLUTION + 0.5;
    p3.y = p1.y;

    p4.x = p3.x;
    p4.y = p2.y;

    int width = p4.x + board_size;
    int height = p4.y + board_size;
    Mat model = Mat::zeros(height, width, CV_8UC1);

    cv::line(model, p1, p2, cv::Scalar(255));
    cv::line(model, p1, p3, cv::Scalar(255));
    // cv::line(model, p2, p4, cv::Scalar(255));
    cv::line(model, p3, p4, cv::Scalar(255));

    GaussianBlur(model, model, Size(0, 0), 5);
    cv::normalize(model, model, 0, 255, NORM_MINMAX, CV_8UC1);

    std::string image_path = getenv("HOME") + std::string("/test1.png");
    imwrite(image_path, model, compression_params);
  }
  /*
    // test2
    {
      float w = 0.80, h = 0.80;
      cv::Point p1, p2, p3, p4;
      p1.x = board_size;
      p1.y = board_size;

      p2.x = p1.x;
      p2.y = p1.y + h / RESOLUTION + 0.5;

      p3.x = p1.x + w / RESOLUTION + 0.5;
      p3.y = p1.y;

      p4.x = p3.x;
      p4.y = p2.y;

      int width = p4.x + board_size;
      int height = p4.y + board_size;
      Mat model = Mat::zeros(height, width, CV_8UC1);

      cv::line(model, p1, p2, cv::Scalar(255));
      cv::line(model, p1, p3, cv::Scalar(255));
      cv::line(model, p2, p4, cv::Scalar(255));
      cv::line(model, p3, p4, cv::Scalar(255));


      GaussianBlur(model, model, Size(0, 0), 5);
      cv::normalize(model, model, 0, 255, NORM_MINMAX, CV_8UC1);

      std::string image_path = getenv("HOME") + std::string("/test2.png");
      imwrite(image_path, model, compression_params);
    }
    /**/
}

// 矩形货架，四个角
Mat template_shelf_RECT_4_FOOT(double L1, double L2, double L3, double L4) {
  std::cout << "template_shelf_RECT_4_FOOT\n"
            << "L1: " << L1 << ", L2: " << L2 << ", L3: " << L3
            << ", L4: " << L4 << std::endl;

  Point p1, p2, p3, p4;

  int board_size = 25;
  int half_rect_width = (L4 / 2) / RESOLUTION + 0.5;
  int half_rect_height = (L3 / 2) / RESOLUTION + 0.5;

  // outer rectangle, p1~p4 all rectangle certers
  p1.x = board_size + half_rect_width;
  p1.y = board_size + half_rect_height;

  p2.x = p1.x;
  p2.y = p1.y + 2 * half_rect_height + (int)(L1 / RESOLUTION + 0.5);

  p3.x = p1.x + 2 * half_rect_width + (int)(L2 / RESOLUTION + 0.5);
  p3.y = p1.y;

  p4.x = p2.x + 2 * half_rect_width + (int)(L2 / RESOLUTION + 0.5);
  p4.y = p2.y;

  int width = p4.x + board_size + half_rect_width;
  int height = p4.y + board_size + half_rect_height;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
           cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Point(p1.x + half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x + half_rect_width, p2.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
           cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p3.x - half_rect_width, p3.y + half_rect_height),
           cv::Point(p3.x + half_rect_width, p3.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x + half_rect_width, p4.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p3.x - half_rect_width, p3.y - half_rect_height),
  // cv::Point(p3.x +
  // half_rect_width, p3.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p4.x - half_rect_width, p4.y - half_rect_height),
           cv::Point(p4.x - half_rect_width, p4.y + half_rect_height),
           cv::Scalar(255));

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);
  return model.clone();
}
// 矩形货架，2个角
Mat template_shelf_RECT_2_FOOT(double L1, double L3, double L4) {
  std::cout << "template_shelf_RECT_2_FOOT\n"
            << "L1: " << L1 << ", L3: " << L3 << ", L4: " << L4 << std::endl;

  Point p1, p2;

  int board_size = 25;
  int half_rect_width = (L4 / 2) / RESOLUTION + 0.5;
  int half_rect_height = (L3 / 2) / RESOLUTION + 0.5;

  // outer rectangle, p1~p4 all rectangle certers
  p1.x = board_size + half_rect_width;
  p1.y = board_size + half_rect_height;

  p2.x = p1.x;
  p2.y = p1.y + 2 * half_rect_height + (int)(L1 / RESOLUTION + 0.5);

  int width = p2.x + board_size + half_rect_width;
  int height = p2.y + board_size + +half_rect_height;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
           cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p1.x - half_rect_width, p1.y + half_rect_height),
           cv::Point(p1.x + half_rect_width, p1.y + half_rect_height),
           cv::Scalar(255));

  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x + half_rect_width, p2.y - half_rect_height),
           cv::Scalar(255));
  // cv::line(model, cv::Point(p1.x - half_rect_width, p1.y - half_rect_height),
  // cv::Point(p1.x +
  // half_rect_width, p1.y - half_rect_height), cv::Scalar(255));
  cv::line(model, cv::Point(p2.x - half_rect_width, p2.y - half_rect_height),
           cv::Point(p2.x - half_rect_width, p2.y + half_rect_height),
           cv::Scalar(255));

  GaussianBlur(model, model, Size(0, 0), 7);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);
  return model.clone();
}

// 圆形货架，8个脚
Mat template_shelf_CIRCLE_8_FOOT(double L1, double L2, double L3, double R) {
  Point p1, p2, p3, p4, p11, p22, p33, p44;
  int board_size = 25;

  int radius = R / RESOLUTION + 0.5;

  p1.x = board_size + radius;
  p1.y = board_size + radius;
  p2.x = p1.x;
  p2.y = p1.y + (int)(L1 / RESOLUTION + 0.5);

  p3.x = p1.x + (int)(L2 / RESOLUTION + 0.5);
  p3.y = p1.y;
  p4.x = p2.x + (int)(L2 / RESOLUTION + 0.5);
  p4.y = p2.y;

  double interal = (L1 - L3) / 2;
  p11.x = p1.x;
  p11.y = p1.y + interal / RESOLUTION + 0.5;
  p22.x = p2.x;
  p22.y = p2.y - interal / RESOLUTION + 0.5;

  p33.x = p11.x + (int)(L2 / RESOLUTION + 0.5);
  p33.y = p11.y;
  p44.x = p22.x + (int)(L2 / RESOLUTION + 0.5);
  p44.y = p22.y;

  int width = p4.x + board_size + radius;
  int height = p4.y + board_size + radius;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  circle(model, p1, radius, Scalar(255), 2);
  circle(model, p2, radius, Scalar(255), 2);
  circle(model, p3, radius, Scalar(255), 2);
  circle(model, p4, radius, Scalar(255), 2);

  circle(model, p11, radius, Scalar(255), 2);
  circle(model, p22, radius, Scalar(255), 2);
  circle(model, p33, radius, Scalar(255), 2);
  circle(model, p44, radius, Scalar(255), 2);

  GaussianBlur(model, model, Size(0, 0), 5);

  cv::normalize(model, model, 0, 255, NORM_MINMAX, CV_8UC1);
  return model.clone();
}

// 圆形货架，4个脚
Mat template_shelf_CIRCLE_4_FOOT(double L1, double L2, double R) {
  Point p1, p2, p3, p4;
  int board_size = 25;

  int radius = R / RESOLUTION + 0.5;

  p1.x = board_size + radius;
  p1.y = board_size + radius;
  p2.x = p1.x;
  p2.y = p1.y + (int)(L1 / RESOLUTION + 0.5);

  p3.x = p1.x + (int)(L2 / RESOLUTION + 0.5);
  p3.y = p1.y;
  p4.x = p2.x + (int)(L2 / RESOLUTION + 0.5);
  p4.y = p2.y;

  int width = p4.x + board_size + radius;
  int height = p4.y + board_size + radius;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  circle(model, p1, radius, Scalar(255), 2);
  circle(model, p2, radius, Scalar(255), 2);
  circle(model, p3, radius, Scalar(255), 2);
  circle(model, p4, radius, Scalar(255), 2);

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);

  return model.clone();
}
// 圆形货架，2个脚
Mat template_shelf_CIRCLE_2_FOOT(double L1, double R) {
  Point p1, p2;
  int board_size = 25;

  int radius = R / RESOLUTION + 0.5;

  p1.x = board_size + radius;
  p1.y = board_size + radius;
  p2.x = p1.x;
  p2.y = p1.y + (int)(L1 / RESOLUTION + 0.5);

  int width = p2.x + board_size + radius;
  int height = p2.y + board_size + radius;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  circle(model, p1, radius, Scalar(255), 2);
  circle(model, p2, radius, Scalar(255), 2);

  GaussianBlur(model, model, Size(0, 0), 7);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);

  return model.clone();
}

Mat generate_rect_shelf(float width, float height) {
  Point p1, p2, p3, p4;
  int board_size = 25;

  p1.x = board_size;
  p1.y = board_size;

  p2.x = p1.x + (int)(height / RESOLUTION + 0.5);
  p2.y = p1.y;

  p3.x = p2.x;
  p3.y = p2.y + (int)(width / RESOLUTION + 0.5);

  p4.x = p1.x;
  p4.y = p3.y;

  int width_pic = p3.x + board_size;
  int height_pic = p3.y + board_size;
  Mat model = Mat::zeros(height_pic, width_pic, CV_8UC1);
  line(model, p1, p2, Scalar(255));
  line(model, p2, p3, Scalar(255));
  line(model, p3, p4, Scalar(255));
  line(model, p4, p1, Scalar(255));

  GaussianBlur(model, model, Size(0, 0), 5);

  double minVal = 0, maxVal = 255;
  minMaxLoc(model, &minVal, &maxVal);
  model = model * 255 / int(maxVal);

  return model.clone();
}

Mat template_chargingPile_aurora() {
  Point p1, p2, p3, p4, p5;
  int board_size = 25;
  p1.x = board_size;
  p1.y = board_size;
  p2.x = p1.x;
  p3.x = int(0.182 / RESOLUTION) + p1.x;
  p4.x = p1.x;
  p5.x = p1.x;

  p2.y = int(0.022 / RESOLUTION) + p1.y;
  p3.y = int(0.2035 / RESOLUTION) + p1.y;
  p4.y = int(0.385 / RESOLUTION) + p1.y;
  p5.y = int(0.407 / RESOLUTION) + p1.y;

  int width = p3.x + board_size;
  int height = p5.y + board_size;

  std::cout << "width: " << width << ", height: " << height << std::endl
            << std::flush;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  line(model, p1, p2, Scalar(255));
  line(model, p2, p3, Scalar(255));
  line(model, p3, p4, Scalar(255));
  line(model, p4, p5, Scalar(255));
  GaussianBlur(model, model, Size(0, 0), 5);
  model = model * 5;
  model = model * 255 / 80;

  return model.clone();
}

Mat generate_pallet() {
  Point p1, p2, p3, p4, p5, p6;
  Point p11, p22, p33, p44, p55, p66;
  int board_size = 25;
  float leg_width = 0.087f;
  float leg_height = 0.087f;
  float leg_interal = 0.3725;
  p1.x = p2.x = p3.x = p4.x = p5.x = p6.x = board_size;
  p1.y = board_size;
  p2.y = p1.y + int(leg_width / RESOLUTION);
  p3.y = p2.y + int(leg_interal / RESOLUTION);
  p4.y = p3.y + int(leg_width / RESOLUTION);
  p5.y = p4.y + int(leg_interal / RESOLUTION);
  p6.y = p5.y + int(leg_width / RESOLUTION);

  p11.x = p22.x = p33.x = p44.x = p55.x = p66.x =
      board_size + int(leg_height / RESOLUTION);
  p11.y = board_size;
  p22.y = p11.y + int(leg_width / RESOLUTION);
  p33.y = p22.y + int(leg_interal / RESOLUTION);
  p44.y = p33.y + int(leg_width / RESOLUTION);
  p55.y = p44.y + int(leg_interal / RESOLUTION);
  p66.y = p55.y + int(leg_width / RESOLUTION);

  int width = p66.x + board_size;
  int height = p66.y + board_size;

  std::cout << "width: " << width << ", height: " << height << std::endl
            << std::flush;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  line(model, p1, p2, Scalar(255));
  line(model, p3, p4, Scalar(255));
  line(model, p5, p6, Scalar(255));
  line(model, p1, p11, Scalar(255));
  line(model, p2, p22, Scalar(255));
  line(model, p3, p33, Scalar(255));
  line(model, p4, p44, Scalar(255));
  line(model, p5, p55, Scalar(255));
  line(model, p6, p66, Scalar(255));
  GaussianBlur(model, model, Size(0, 0), 5);
  model = model * 5;
  model = model * 255 / 80;

  return model.clone();
}

Mat generate_haixin() {
  Point p1, p2, p3, p4;
  Point x_offset, y_offset;

  float leg_width = 0.027f;
  float leg_height = 0.027f;

  int leg_w = int(leg_width / (RESOLUTION));
  int leg_h = int(leg_height / (RESOLUTION));

  int board_size = 25;
  p1.x = p2.x = board_size + leg_w / 2;
  p1.y = board_size + leg_h / 2;
  p2.y = p1.y + int(1.34 / RESOLUTION);

  p3.x = p4.x = p1.x + int((1.75 / RESOLUTION));
  p3.y = p1.y + int(((1.34 - 0.73) / 2) / RESOLUTION);
  p4.y = p3.y + int(0.73 / RESOLUTION);

  x_offset = Point(leg_w / 2, 0);
  y_offset = Point(0, leg_h / 2);

  int width = p3.x + x_offset.x + board_size;
  int height = p2.y + y_offset.y + board_size;

  std::cout << "width: " << width << ", height: " << height << std::endl
            << std::flush;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  line(model, p1 - x_offset + y_offset, p1 + x_offset + y_offset, Scalar(255));
  line(model, p1 - x_offset - y_offset, p1 - x_offset + y_offset, Scalar(255));
  line(model, p2 - x_offset - y_offset, p2 + x_offset - y_offset, Scalar(255));
  line(model, p2 - x_offset - y_offset, p2 - x_offset + y_offset, Scalar(255));

  line(model, p3 - x_offset + y_offset, p3 + x_offset + y_offset, Scalar(255));
  line(model, p3 - x_offset - y_offset, p3 - x_offset + y_offset, Scalar(255));
  line(model, p4 - x_offset - y_offset, p4 + x_offset - y_offset, Scalar(255));
  line(model, p4 - x_offset - y_offset, p4 - x_offset + y_offset, Scalar(255));
  GaussianBlur(model, model, Size(0, 0), 5);
  model = model * 5;
  model = model * 255 / 80;

  return model.clone();
}

Mat generate_chengdubili() {
  int board_size = 25;
  Point offset(board_size, board_size);
  Point2d phy_p1, phy_p2, phy_p3;
  double l1 = 0.18, l2 = 0.125, l3 = 0.25, l4 = 0.4, l5 = 0.92;

  phy_p1 = Point2d(l2 / 2, l1 / 2);
  phy_p2 = Point2d((l5 + l2 / 2), l1 / 2);
  phy_p3 = Point2d(l5 / 2 + l2 / 2, l1 / 2 + 3.12);

  Point p1_0, p1_1, p1_2;
  p1_0 = Point(0, 0) + offset;
  p1_1 = Point(0, l1 / RESOLUTION + 0.5) + offset;
  p1_2 = Point(l2 / RESOLUTION + 0.5, l1 / RESOLUTION + 0.5) + offset;

  Point p2_0, p2_1, p2_2;
  p2_0 = Point(l5 / RESOLUTION + 0.5, 0) + offset;
  p2_1 = Point(l5 / RESOLUTION + 0.5, l1 / RESOLUTION + 0.5) + offset;
  p2_2 = Point((l5 + l2) / RESOLUTION + 0.5, l1 / RESOLUTION + 0.5) + offset;

  Point p3_0, p3_1, p3_2;
  p3_0 = Point((phy_p3.x - l4 / 2) / RESOLUTION + 0.5,
               (phy_p3.y - l3 / 2) / RESOLUTION + 0.5) +
         offset;
  p3_1 = Point((phy_p3.x + l4 / 2) / RESOLUTION + 0.5,
               (phy_p3.y - l3 / 2) / RESOLUTION + 0.5) +
         offset;
  p3_2 = Point((phy_p3.x - l4 / 2) / RESOLUTION + 0.5,
               (phy_p3.y + l3 / 2) / RESOLUTION + 0.5) +
         offset;

  int width = p2_2.x + board_size;
  int height = p3_2.y + board_size;

  std::cout << "width: " << width << ", height: " << height << std::endl
            << std::flush;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  line(model, p1_0, p1_1, Scalar(255));
  line(model, p1_1, p1_2, Scalar(255));
  line(model, p2_0, p2_1, Scalar(255));
  line(model, p2_1, p2_2, Scalar(255));
  line(model, p3_0, p3_1, Scalar(255));
  line(model, p3_0, p3_2, Scalar(255));

  GaussianBlur(model, model, Size(0, 0), 5);
  model = model * 5;
  model = model * 255 / 80;

  return model.clone();
}

// 20240920江阴海达异形货架,左侧两腿，右侧一腿方向
Mat generate_jiangyinhaida1() {
  int board_size = 25;
  Point offset(board_size, board_size);

  double l1 = 0.122, l2 = 0.096, l3 = 0.12, l4 = 0.32, l5 = 0.5, l6 = 1.49;
  double w = l5 + l1 + l4;
  double h = l6;

  Point p1_0, p1_1, p1_2;
  p1_0 = Point(l5 / RESOLUTION + 0.5, (l6 - l3) / RESOLUTION + 0.5) + offset;
  p1_1 = Point((l5 + l1) / RESOLUTION + 0.5, (l6 - l3) / RESOLUTION + 0.5) +
         offset;
  p1_2 = Point(l5 / RESOLUTION + 0.5, l6 / RESOLUTION + 0.5) + offset;

  Point p2_0, p2_1, p2_2;
  p2_0 = Point((w - l2) / RESOLUTION + 0.5, l2 / RESOLUTION + 0.5) + offset;
  p2_1 = Point(w / RESOLUTION + 0.5, l2 / RESOLUTION + 0.5) + offset;
  p2_2 = Point((w - l2) / RESOLUTION + 0.5, 0) + offset;

  Point p3_0, p3_1, p3_2;
  p3_0 = Point(0, l3 / RESOLUTION + 0.5) + offset;
  p3_1 = Point(l3 / RESOLUTION + 0.5, l3 / RESOLUTION + 0.5) + offset;
  p3_2 = Point(0, 0) + offset;

  int width = w / RESOLUTION + 0.5 + board_size * 2;
  int height = h / RESOLUTION + 0.5 + board_size * 2;

  std::cout << "width: " << width << ", height: " << height << std::endl
            << std::flush;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  line(model, p1_0, p1_1, Scalar(255));
  line(model, p1_0, p1_2, Scalar(255));
  line(model, p2_0, p2_1, Scalar(255));
  line(model, p2_0, p2_2, Scalar(255));
  line(model, p3_0, p3_1, Scalar(255));
  line(model, p3_0, p3_2, Scalar(255));

  GaussianBlur(model, model, Size(0, 0), 5);
  model = model * 5;
  model = model * 255 / 80;

  return model.clone();
}

// 20240920江阴海达异形货架
Mat generate_jiangyinhaida2() {
  int board_size = 25;
  Point offset(board_size, board_size);

  double l1 = 0.122, l2 = 0.096, l3 = 0.12, l4 = 0.32, l5 = 0.5, l6 = 1.49;
  double w = l5 + l1 + l4;
  double h = l6;

  Point p1_0, p1_1, p1_2;
  p1_0 = Point(l5 / RESOLUTION + 0.5, l1 / RESOLUTION + 0.5) + offset;
  p1_1 = Point(l5 / RESOLUTION + 0.5, 0) + offset;
  p1_2 = Point((l5 + l1) / RESOLUTION + 0.5, l1 / RESOLUTION + 0.5) + offset;

  Point p2_0, p2_1, p2_2;
  p2_0 =
      Point((w - l2) / RESOLUTION + 0.5, (l6 - l2) / RESOLUTION + 0.5) + offset;
  p2_1 = Point(w / RESOLUTION + 0.5, l2 / RESOLUTION + 0.5) + offset;
  p2_2 = Point((w - l2) / RESOLUTION + 0.5, 0) + offset;

  Point p3_0, p3_1, p3_2;
  p3_0 = Point(0, l3 / RESOLUTION + 0.5) + offset;
  p3_1 = Point(l3 / RESOLUTION + 0.5, l3 / RESOLUTION + 0.5) + offset;
  p3_2 = Point(0, 0) + offset;

  int width = w / RESOLUTION + 0.5 + board_size * 2;
  int height = h / RESOLUTION + 0.5 + board_size * 2;

  std::cout << "width: " << width << ", height: " << height << std::endl
            << std::flush;
  Mat model = Mat::zeros(height, width, CV_8UC1);

  line(model, p1_0, p1_1, Scalar(255));
  line(model, p1_0, p1_2, Scalar(255));
  line(model, p2_0, p2_1, Scalar(255));
  line(model, p2_0, p2_2, Scalar(255));
  line(model, p3_0, p3_1, Scalar(255));
  line(model, p3_0, p3_2, Scalar(255));

  GaussianBlur(model, model, Size(0, 0), 5);
  model = model * 5;
  model = model * 255 / 80;

  return model.clone();
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_logbufsecs = 0;

  template_generate_test(argc, argv);
  return 0;

  {
    Mat model = generate_jiangyinhaida1();

    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);

    // std::string image_path = getenv("HOME") +
    // std::string("/probability_grid.png");
    std::string image_path = getenv("HOME") + std::string("/hd.png");
    imwrite(image_path, model, compression_params);

    std::cout << "Create succeed." << std::endl;
  }
  return 1;

  ros::init(argc, argv, "template_generator_node");
  ros::start();

  float width, height;
  if (!ros::param::get("~width", width)) {
    std::cout << "No width Given, exit.";
    exit(0);
  }

  if (!ros::param::get("~height", height)) {
    std::cout << "No height Given, exit.";
    exit(0);
  }
  Mat model = generate_rect_shelf(width, height);

  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(0);

  std::string image_path = getenv("HOME") + std::string("/test.png");
  imwrite(image_path, model, compression_params);

  std::cout << "Create succeed." << std::endl;
  ros::shutdown();
  return 0;
}
