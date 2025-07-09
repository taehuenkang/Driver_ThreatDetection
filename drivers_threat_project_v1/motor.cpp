#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

class MotorControlNode : public rclcpp::Node
{
public:
    MotorControlNode()
    : Node("motor_control_node"), threat_detected_(false)
    {
        // 퍼블리셔 QoS: Reliable로 맞춤
        auto qos = rclcpp::QoS(10).reliable();
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", qos);

        subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "/threat", 10,
            std::bind(&MotorControlNode::threat_callback, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MotorControlNode::control_loop, this)
        );
    }

private:
    void threat_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        threat_detected_ = msg->data;
    }

    void control_loop()
    {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->now();  // 반드시 timestamp!
        if (threat_detected_) {
            cmd.twist.linear.x = 0.0;
            cmd.twist.angular.z = 0.0;
            RCLCPP_WARN(this->get_logger(), "⚠️ 위협 감지됨: 정지 명령 전송 중...");
        } else {
            cmd.twist.linear.x = 0.2;
            cmd.twist.angular.z = 0.0;
            //RCLCPP_INFO(this->get_logger(), "전진 명령 전송 중...");
        }
        publisher_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool threat_detected_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControlNode>());
    rclcpp::shutdown();
    return 0;
}
