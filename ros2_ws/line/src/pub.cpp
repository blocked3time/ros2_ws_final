#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "geometry_msgs/msg/vector3.hpp"
//#include "dxl/dxl.hpp"
#include <memory>
#include <chrono>
#include <functional>
#include <thread>
#include <chrono>
#include<math.h>
#define SPEED 75
#define GAIN 0.15 //0.25
#define MINDISTANCE 100
using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace cv;
void setFrame(Mat& frame){
	frame = frame(Rect(Point(0,frame.rows/4*3),Point(frame.cols,frame.rows)));  
	cvtColor(frame,frame,COLOR_RGB2GRAY);
	frame +=  Scalar(100) - mean(frame);
	threshold(frame, frame, 0, 255, THRESH_BINARY | THRESH_OTSU);
}

int findMinIndex(cv::Mat stats,cv::Mat centroids, int lable, cv::Point& po,int Min){
	int index = 0;
	double mindistance = Min;
	for(int i = 1;i<lable;i++){
        if(stats.at<int>(i,4)<50) continue;
		double distance = sqrt(pow((po.x-centroids.at<double>(i,0)),2)+pow((po.y- centroids.at<double>(i,1)),2));
		if(distance<mindistance){
            mindistance = distance;
            index = i;
        }
     }
	if(mindistance <  MINDISTANCE && po != Point(centroids.at<double>(index,0),centroids.at<double>(index,1))) 
		po = Point(centroids.at<double>(index,0),centroids.at<double>(index,1));
	return index;
}
void drawBoundingBox(cv::Mat& frame,cv::Mat stats,cv::Mat centroids, int lable, int index, cv::Point po){
	cvtColor(frame, frame, COLOR_GRAY2BGR);
	Scalar sc;
	 for(int i = 1;i<lable;i++){
		if(i == index) sc = Scalar(0,0,255);
        else sc =stats.at<int>(i,4)<50 ? Scalar(0,255,255):Scalar(255,0,0);
        rectangle(frame,Rect(stats.at<int>(i,0),stats.at<int>(i,1),stats.at<int>(i,2),stats.at<int>(i,3)),sc,2);
        rectangle(frame,Rect(centroids.at<double>(i,0),centroids.at<double>(i,1),3,3),sc,4);
        } 
	if(index == 0){
		 rectangle(frame,Rect(po.x,po.y,3,3),Scalar(0,0,255),2);
	}
}
class dd : public rclcpp::Node{
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub;
    geometry_msgs::msg::Vector3 vel;
   // rclcpp::TimerBase::SharedPtr timer_;
    void subcallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg){
        cv::Mat frame, stats, centroids, labels;
        frame = cv::imdecode(cv::Mat(msg->data),  cv::IMREAD_COLOR);
        cv::imshow("wsl",frame);

        RCLCPP_INFO(this->get_logger(), "Received Image : %s,%d,%d", msg->format.c_str(),frame.rows,frame.cols);

        setFrame(frame);
        static cv::Point po = cv::Point(frame.cols/2,frame.rows/2);
        int lable =  connectedComponentsWithStats(frame, labels, stats, centroids);
        int index = findMinIndex(stats,centroids, lable,po, MINDISTANCE);
        drawBoundingBox(frame,stats,centroids, lable, index,po);
        //RCLCPP_INFO(this->get_logger(), "index : %d",lable);
        vel.y =  -1*SPEED -  (frame.cols/2 -po.x)*GAIN;
        vel.x = SPEED -  (frame.cols/2 -po.x)*GAIN;
        pub->publish(vel);
        cv::imshow("wsl",frame);
        cv::waitKey(1);
        }
    public:
    dd(std::string s): Node(s){
        vel.x = 0;
        vel.y = 0;
        vel.z = 0;
        pub = this->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", rclcpp::QoS(rclcpp::KeepLast(10)));
        sub = this->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", rclcpp::QoS(
            rclcpp::KeepLast(10))/*.best_effort()*/,std::bind(&dd::subcallback, this, _1));
     //   timer_ = this->create_wall_timer(40ms,std::bind(&dd::pubcallback ,this));
    }
};
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dd>("node_dxlpub");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
