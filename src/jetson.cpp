#include "line_tracer/jetson.hpp"

std::string src = "nvarguscamerasrc sensor-id=0 ! \
	video/x-raw(memory:NVMM), width=(int)640, height=(int)360, \
    format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, \
    width=(int)640, height=(int)360, format=(string)BGRx ! \
	videoconvert ! video/x-raw, format=(string)BGR ! appsink"; 
    
Pub::Pub() : Node("campub")
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile );
    timer_ = this->create_wall_timer(25ms, std::bind(&Pub::publish_msg, this));
    
    cv::VideoCapture cap(src, cv::CAP_GSTREAMER);

    // cap.open("/home/jetson/ros2_ws/src/line_tracer/src/8_lt_cw_100rpm_in.mp4"); //영상
    if (!cap.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open video!");
        rclcpp::shutdown();
        return;
    }

    if(!dxl.open())
    {
        RCLCPP_ERROR(this->get_logger(), "dynamixel open error");
        rclcpp::shutdown();
        //return -1;
    } 
    std::function<void(const std_msgs::msg::Int32::SharedPtr msg)> fn;
    fn = std::bind(&Pub::mysub_callback, this, dxl, _1);
    sub_ = this->create_subscription<std_msgs::msg::Int32>("err", qos_profile,fn);
}
void Pub::publish_msg()
{ 
    cap >> frame;
    if (frame.empty()) { RCLCPP_ERROR(this->get_logger(), "frame empty"); return;}
    msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg();
    pub_->publish(*msg);
}

void Pub::mysub_callback(Dxl& mdxl, const std_msgs::msg::Int32::SharedPtr intmsg)
{
    err = intmsg->data;
    lvel = 100 - gain * err;//왼쪽 바퀴 속도
	rvel = -(100 + gain * err);//오른쪽 바퀴 속도
    RCLCPP_INFO(this->get_logger(), "Received message: %d %d", lvel, rvel);
    mdxl.setVelocity(lvel, rvel);
}