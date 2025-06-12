#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
using std::placeholders::_1;
cv::VideoWriter writer2;
void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data),  cv::IMREAD_COLOR);
    cv::imshow("wsl2",frame);
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    cv::imshow("wsl3",frame);
    cv::threshold(frame, frame, 128, 255, cv::THRESH_BINARY);
    cv::imshow("wsl",frame);
    cv::waitKey(1);
    RCLCPP_INFO(node->get_logger(), "Received Image : %s,%d,%d", msg->format.c_str(),frame.rows,frame.cols);
}
void mypub_callback2(rclcpp::Node::SharedPtr node, const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    static int goal1=0, goal2=0;
    char c=getchar();
    switch(c)
        {
        case 's' : goal1=0; goal2=0; break;
        case ' ' : goal1=0; goal2=0; break;
        case 'f': goal1=50; goal2=-50; break;
        case 'b': goal1=-50; goal2=50; break;
        case 'l': goal1=-50; goal2=-50; break;
        case 'r': goal1=50; goal2=50; break;
        }
    msg.x = goal1;
    msg.x = goal2;
    node->publish(msg);
}
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub_wsl");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));//.best_effort();
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);
    auto mysub = node->create_subscription<geometry_msgs::msg::Vector3>("topic_dxlpub", qos_profile);
     fn = std::bind(mypub_callback, node, _1);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

