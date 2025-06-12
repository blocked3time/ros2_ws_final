#include"rclcpp/rclcpp.hpp"
#include"geometry_msgs/msg/vector3.hpp"
#include <unistd.h>
#include<memory>

 int main(int argc, char*argv[])
 {
    rclcpp::init(argc, argv);
    auto node=std::make_shared<rclcpp::Node>("node_dxlpub");
    auto qos_profile=rclcpp::QoS(rclcpp::KeepLast(10));
    auto mypub=node->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", qos_profile);
    geometry_msgs::msg::Vector3 vel;
    vel.x=0;
    vel.y=0;
    vel.z=0;
    rclcpp::WallRate loop_rate(20.0); //20hz->50msec
    int vel1=0,vel2=0;
    int goal1=0,goal2=0;
    while(rclcpp::ok())
        {
        if (1)
        {
            char c=getchar();
            switch(c)
                {
                case 's' : goal1=0; goal2=0; break;
                case ' ' : goal1=0; goal2=0; break;
                case 'f': goal1=50; goal2=-50; break;
                case 'b': goal1=-50; goal2=50; break;
                case 'l': goal1=-50; goal2=-50; break;
                case 'r': goal1=50; goal2=50; break;
              // default : goal1=0; goal2=0; break;
            } 
        }

    vel.x=vel1 = goal1;
    vel.y=vel2= goal2; 
    RCLCPP_INFO(node->get_logger(), "Publish: %lf,%lf", vel.x, vel.y);
    mypub->publish(vel);
    loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}