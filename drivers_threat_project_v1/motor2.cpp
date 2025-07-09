#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode()
    : Node("motor_control_node"),
      threat_detected_(false),
      current_lane_(1),
      lane_changed_(false),
      is_lane_changing_(false),
      lane_change_duration_(3.5) // 차선 변경 시간(초)
    {
        auto qos = rclcpp::QoS(10).reliable();
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", qos);

        threat_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/threat", 10,
            std::bind(&MotorControlNode::threat_callback, this, std::placeholders::_1)
        );
        lane_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/lane_info", 10,
            std::bind(&MotorControlNode::lane_callback, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MotorControlNode::control_loop, this)
        );
    }

private:
    void threat_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !threat_detected_) {
            threat_detected_ = true;
            if (current_lane_ == 1) {
                is_lane_changing_ = true;
                lane_change_start_time_ = this->now();
                RCLCPP_WARN(this->get_logger(), "[위협 감지] 1차선→2차선 차선변경 시작!");
            } else {
                is_lane_changing_ = false;
                lane_changed_ = true;
                RCLCPP_WARN(this->get_logger(), "[위협 감지] 이미 2차선! 멈춤!");
            }
        } else if (!msg->data && threat_detected_) {
            threat_detected_ = false;
            lane_changed_ = false;
            is_lane_changing_ = false;
            RCLCPP_INFO(this->get_logger(), "위협 해제: 상태 초기화, 정상 전진");
        }
    }

    void lane_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (!is_lane_changing_) {
            current_lane_ = msg->data;
        }
    }

    void control_loop()
    {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->now();

        if (threat_detected_) {
            if (is_lane_changing_) {
                double elapsed = (this->now() - lane_change_start_time_).seconds();

                if (elapsed < 2.5) {
                    // 1단계: 우측 비스듬히
                    cmd.twist.linear.x = 0.18;
                    cmd.twist.angular.z = -0.2;
                    RCLCPP_WARN(this->get_logger(),
                        "[1/3] 우측 비스듬히 (%.1fs/2.5s)", elapsed);
                }
                else if (elapsed < 4.0) {
                    // 2단계: 좌측 비스듬히
                    cmd.twist.linear.x = 0.18;
                    cmd.twist.angular.z = 0.18;
                    RCLCPP_WARN(this->get_logger(),
                        "[2/3] 좌측 비스듬히 (%.1fs/1.5s)", elapsed - 2.5);
                }
                else if (elapsed < 5.5) {
                    // 3단계: 직진
                    cmd.twist.linear.x = 0.18;
                    cmd.twist.angular.z = 0.0;
                    RCLCPP_WARN(this->get_logger(),
                        "[3/3] 직진 (%.1fs/1.5s)", elapsed - 4.0);
                }
                else {
                    // 정지
                    is_lane_changing_ = false;
                    lane_changed_ = true;
                    cmd.twist.linear.x = 0.0;
                    cmd.twist.angular.z = 0.0;
                    RCLCPP_WARN(this->get_logger(), "차선 변경 완료 후 멈춤!");
                }
            } else {
                cmd.twist.linear.x = 0.0;
                cmd.twist.angular.z = 0.0;
                RCLCPP_WARN(this->get_logger(), "위협 감지 상태! 멈춤(2차선 혹은 변경 완료)");
            }
        } else {
            cmd.twist.linear.x = 0.2;
            cmd.twist.angular.z = 0.0;
        }
        publisher_->publish(cmd);
    }

    // 멤버 변수들
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr threat_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lane_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool threat_detected_;
    int current_lane_;
    bool lane_changed_;
    bool is_lane_changing_;
    rclcpp::Time lane_change_start_time_;
    const double lane_change_duration_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControlNode>());
    rclcpp::shutdown();
    return 0;
}