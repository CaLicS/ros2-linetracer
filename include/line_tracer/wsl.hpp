#ifndef _VM_HPP_
#define _VM_HPP_
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/int32.hpp"
#include "line_tracer/dxl.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
#include <queue>
#include <ctime>
#include <unistd.h>
#include <signal.h>
#include <chrono>
#include <math.h>
#include <vector>
using namespace std::chrono_literals;
using std::placeholders::_1;
class Sub : public rclcpp::Node
{
    private:
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
        void mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
        std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;

        std::vector<double>vec_x = {0,0};
        std::vector<double>vec_y = {0,0};

        cv::Mat frame, color, dst;
        cv::Mat labels, stats, centroids;

        double* p;
        int* ps;

        double cur_min = 0;
        double now_min = 0;
        double gain = 0.3;
        double a = 0, b = 0;

        int cnt = 0;
        int lvel = 0, rvel = 0;
        int vel1, vel2;

        bool first = true;
        
        cv::VideoWriter writer1;
        cv::VideoWriter writer2;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
        std_msgs::msg::Int32 intmsg;
        void publish_msg();
    public:
        Sub();
        int err;
};
#endif //_SUB_HPP_