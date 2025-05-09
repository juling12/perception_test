/*
 * @Author: juling julinger@qq.com
 * @Date: 2025-05-08 15:30:16
 * @LastEditors: juling julinger@qq.com
 * @LastEditTime: 2025-05-08 16:02:51
 */
#include <glog/logging.h>

#include <opencv2/opencv.hpp>

int main() {
  // 1. 读取原始图像
  cv::Mat img = cv::imread("/home/juling/4_color.bmp");
  if (img.empty()) {
    std::cerr << "图像读取失败！" << std::endl;
    return -1;
  }

  // 2. 计算旋转中心（图像中心）
  cv::Point2f center(img.cols / 2.0f, img.rows / 2.0f);

  // 3. 设定旋转角度和缩放因子
  double angle = 90;   // 旋转角度（度）
  double scale = 1.0;  // 缩放比例

  // 4. 获取旋转仿射矩阵
  cv::Mat rot_mat = cv::getRotationMatrix2D(center, angle, scale);
  LOG(INFO) << "rot_mat: \n" << rot_mat;
  cv::Rect max_box = cv::RotatedRect(center, img.size(), angle).boundingRect();
  LOG(INFO) << "max_box: " << max_box;
  rot_mat.at<double>(0, 2) += max_box.width / 2.0 - center.x;
  rot_mat.at<double>(1, 2) += max_box.height / 2.0 - center.y;

  // 5. 根据矩阵进行仿射变换（图像旋转）
  cv::Mat rotated = cv::Mat::zeros(max_box.size(), img.type());
  cv::warpAffine(img, rotated, rot_mat, rotated.size());

  // 6. 显示原图和旋转图像
  cv::imshow("Original", img);
  cv::imshow("Rotated", rotated);
  cv::waitKey(0);
  cv::destroyAllWindows();
  return 0;
}
