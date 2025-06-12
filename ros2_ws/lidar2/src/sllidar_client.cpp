/*
 *  SLLIDAR ROS2 CLIENT
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>
#include "opencv2/opencv.hpp"

#define RAD2DEG(x) ((x)*180./M_PI)

static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
   static cv::VideoWriter writer2("video.mp4", cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 10, cv::Size(500, 500), true);
  cv::Mat lidar(500,500,CV_8UC3,cv::Scalar(255,255,255));
  int count = scan->scan_time / scan->time_increment;
  printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
  printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),
         RAD2DEG(scan->angle_max));
  printf("[SLLIDAR INFO]: i : %f\n",RAD2DEG(scan->angle_increment));
  for (int i = 0; i < count; i++) {
    float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    //printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", degree, scan->ranges[i]);
    double deg_ =(degree)*M_PI/180 ;
    double dis_ =scan->ranges[i];
    double aax = 250+(sin(deg_)*dis_*250/12);
    double aay = 250+(cos(deg_)*dis_*250/12);
    cv::rectangle(lidar,cv::Rect(aax,aay,3,3),cv::Scalar(0,0,255),-1);
    
  }

  cv::rectangle(lidar,cv::Rect(lidar.cols/2-1,lidar.rows/2-1,3,3),cv::Scalar(0,0,0),-1);

  cv::imshow("lidar",lidar);
  writer2<<lidar;
  cv::waitKey(1);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("sllidar_client");

  auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                        "scan", rclcpp::SensorDataQoS(), scanCb);

  rclcpp::spin(node);

  rclcpp::shutdown();


  return 0;
}
