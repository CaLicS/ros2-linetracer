#include "line_tracer/wsl.hpp"
void Sub::mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
	now_min = 0;
	cur_min = 0;
    frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    //gettimeofday(&start, NULL);
	if (frame.empty()) { std::cerr << "frame empty!" << std::endl; return; }

	dst = frame(cv::Rect(0, frame.rows / 4 * 3, frame.cols, 90));
	cv::cvtColor(dst, dst, cv::COLOR_BGR2GRAY);
	dst = dst + (100 - mean(dst)[0]);
	cv::threshold(dst, dst, 130, 255, cv::THRESH_BINARY);
	
	cnt = cv::connectedComponentsWithStats(dst, labels, stats, centroids);
	cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);

	for(int i = 1; i < cnt; i++){
		double* p = centroids.ptr<double>(i);
		int* ps = stats.ptr<int>(i);

		cv::rectangle(dst, cv::Rect(ps[0], ps[1], ps[2], ps[3]), cv::Scalar(255, 0, 0));

	
		if (first && (p[0] > 200 && p[0] < 440 && p[1] > 20)){
			vec_x.insert(vec_x.begin(), p[0]);
			vec_x.push_back(p[0]);

			vec_y.insert(vec_y.begin(), p[1]);
			vec_y.push_back(p[1]);

			cur_min = sqrt((pow(p[0] - (dst.cols / 2.0), 2)) + (pow(p[1] - (dst.rows / 2.0), 2)));
		}
		now_min = sqrt(pow(vec_x.back() - p[0], 2) + pow(vec_y.back() - p[1], 2));
		
		if (i == 1)
			cur_min = now_min;
		
		if ((abs(vec_x.back() - p[0]) <= 160 && abs(vec_y.back() - p[1] <= 50)) && (now_min <= cur_min))
		{
			cur_min = now_min;
			vec_x.insert(vec_x.begin(), p[0]);
			vec_y.insert(vec_y.begin(), p[1]);
			err = dst.cols / 2.0 - p[0];
			cv::rectangle(dst, cv::Rect(ps[0], ps[1], ps[2], ps[3]), cv::Scalar(0, 0, 255));
		}
		
	}
	cv::circle(dst, cv::Point2d(vec_x.front(), vec_y.front()), 4, cv::Scalar(0, 0, 255), -1);
	
	vec_x.push_back(vec_x.front());
	vec_y.push_back(vec_y.front());

	writer1 << frame;
	writer2 << dst;

    cv::imshow("frame", frame);
    cv::imshow("color", dst);
    cv::waitKey(1);

    RCLCPP_INFO(this->get_logger(), "Received Image : %s,%d,%d", msg->format.c_str(),frame.rows,frame.cols);
}
void Sub::publish_msg()
{
    intmsg.data = err;
	RCLCPP_INFO(this->get_logger(), "Publish: %d", intmsg.data);
	pub_->publish(intmsg);
}
Sub::Sub() : Node("camsub_wsl")
{
    writer1.open("output1.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(640, 360));
    writer2.open("output2.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(640, 90));
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile, std::bind(&Sub::mysub_callback, this, _1));
    pub_ = this->create_publisher<std_msgs::msg::Int32>("err", qos_profile);
    timer_ = this->create_wall_timer(50ms, std::bind(&Sub::publish_msg, this));
}
  