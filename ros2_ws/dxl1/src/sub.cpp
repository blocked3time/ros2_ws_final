#include"rclcpp/rclcpp.hpp"
 #include"geometry_msgs/msg/vector3.hpp"
 #include"dxl/dxl.hpp"
 #include<memory>
 #include<functional>
 usin gnamespacestd::placeholders;
 void mysub_callback(rclcpp::Node::SharedPtrnode, Dxl&dxl, const
 geometry_msgs::msg::Vector3::SharedPtrmsg)
 {
 RCLCPP_INFO(node->get_logger(), "Received message: %lf,%lf", msg->x,msg->y);
 dxl.setVelocity((int)msg->x, (int)msg->y);
 }
 int main(intargc, char*argv[])
 {
    rclcpp::init(argc, argv);
    Dxldxl;
    autonode =std::make_shared<rclcpp::Node>("node_dxlsub");
    if(!dxl.open())
    {
    RCLCPP_ERROR(node->get_logger(), "dynamixelopen error");
    rclcpp::shutdown();
    return-1;
    }
    autoqos_profile=rclcpp::QoS(rclcpp::KeepLast(10));
    std::function<void(constgeometry_msgs::msg::Vector3::SharedPtrmsg)>fn;
    fn=std::bind(mysub_callback, node, dxl, _1);
    automysub=node->create_subscription<geometry_msgs::msg::Vector3>("topic_dxlpub",qos_profile,fn);
    rclcpp::spin(node);
    dxl.close();
    rclcpp::shutdown();
    return 0;
 }