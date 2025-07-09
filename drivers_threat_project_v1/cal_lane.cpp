// cal_lane.cpp - ROS2 기반 컨투어 기반 차선 인식 노드 (터미널 출력/이미지 토픽 포함)

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

struct ContourInfo {
    std::vector<cv::Point> pts;
    cv::Rect bbox;
    double width;
    double height;
    double area;
};

class LanePublisherNode : public rclcpp::Node {
public:
    LanePublisherNode() : Node("lane_publisher_node") {
        pub_ = this->create_publisher<std_msgs::msg::Int32>("/lane_info", 10);
        img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/lane_debug_image", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LanePublisherNode::process, this)
        );

        cap_.open("/dev/video0", cv::CAP_V4L2);
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 320);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
        cap_.set(cv::CAP_PROP_FPS, 15);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "카메라 열기 실패");
            rclcpp::shutdown();
        }
    }

private:
    void process() {
        cv::Mat frame;
        if (!cap_.read(frame)) return;

        int h = frame.rows, w = frame.cols, cam_c = w / 2;
        int rect_top = int(h * 0.55), rect_bottom = h;
        std::vector<cv::Point> roi = {
            {0, rect_top}, {w, rect_top}, {w, rect_bottom}, {0, rect_bottom}
        };

        cv::Mat gray, mask, roi_mask = cv::Mat::zeros(frame.size(), CV_8UC1);
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, mask, 95, 255, cv::THRESH_BINARY_INV);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        cv::fillConvexPoly(roi_mask, roi, cv::Scalar(255));
        cv::bitwise_and(mask, roi_mask, mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<ContourInfo> infos;
        for (auto &cnt : contours) {
            cv::Rect r = cv::boundingRect(cnt);
            double w_cnt = r.width, h_cnt = r.height, area = w_cnt * h_cnt;
            int cy = r.y + r.height / 2;
            if (h_cnt < 10 || w_cnt / h_cnt > 0.97 || area < 300.0 || cy < rect_top - 5) continue;
            infos.push_back({cnt, r, w_cnt, h_cnt, area});
        }

        std::sort(infos.begin(), infos.end(), [](const ContourInfo &a, const ContourInfo &b) {
            return a.width > b.width;
        });

        int topN = std::min<int>(3, infos.size());
        std::vector<ContourInfo> topContours(infos.begin(), infos.begin() + topN);

        std::sort(topContours.begin(), topContours.end(), [](const ContourInfo &a, const ContourInfo &b) {
            return (a.bbox.x + a.bbox.width / 2) < (b.bbox.x + b.bbox.width / 2);
        });

        int lane = 0;
        if (topContours.size() >= 2) {
            int blue_center_x = topContours[1].bbox.x + topContours[1].bbox.width / 2;
            lane = (blue_center_x < cam_c) ? 2 : 1;
        }

        // 터미널에 출력 (값이 바뀔 때만)
        static int last_lane = -1;
        if (lane != last_lane) {
            std::cout << "현재 인식 차선: " << lane << "차선" << std::endl;
            last_lane = lane;
        }

        // ROS2 토픽 publish (숫자)
        std_msgs::msg::Int32 msg;
        msg.data = lane;
        pub_->publish(msg);

        // ===== 디버그 이미지 Publish (OpenCV 결과 영상) =====
        // 사각형, 텍스트 등 시각화
        cv::Mat debug = frame.clone();
        for (const auto &c : topContours)
            cv::rectangle(debug, c.bbox, cv::Scalar(0,255,255), 2); // 노란색 사각형 표시
        cv::putText(debug, std::to_string(lane) + "차선", {10,30}, cv::FONT_HERSHEY_SIMPLEX, 1, {0,0,255}, 2);

        // OpenCV -> ROS2 이미지 메시지로 변환 & publish
        auto msg_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug).toImageMsg();
        img_pub_->publish(*msg_img);
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LanePublisherNode>());
    rclcpp::shutdown();
    return 0;
}