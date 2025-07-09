#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <lcd.h>
#include <unistd.h>
#include <string>
#include <cstring>

#define LCD_ADDR 0x27
#define LCD_CHR 1
#define LCD_CMD 0
#define LCD_BACKLIGHT 0x08
#define ENABLE 0b00000100

int fd;

void lcd_toggle_enable(int bits) {
    usleep(500);
    wiringPiI2CWrite(fd, bits | ENABLE);
    usleep(500);
    wiringPiI2CWrite(fd, bits & ~ENABLE);
    usleep(500);
}

void lcd_byte(int bits, int mode) {
    int bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT;
    int bits_low  = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT;

    wiringPiI2CWrite(fd, bits_high);
    lcd_toggle_enable(bits_high);
    wiringPiI2CWrite(fd, bits_low);
    lcd_toggle_enable(bits_low);
}

void lcd_init() {
    lcd_byte(0x33, LCD_CMD);
    lcd_byte(0x32, LCD_CMD);
    lcd_byte(0x06, LCD_CMD);
    lcd_byte(0x0C, LCD_CMD);
    lcd_byte(0x28, LCD_CMD);
    lcd_byte(0x01, LCD_CMD);
    usleep(500);
}

void lcd_loc(int line) {
    if (line == 1)
        lcd_byte(0x80, LCD_CMD);
    else if (line == 2)
        lcd_byte(0xC0, LCD_CMD);
}

void lcd_write(const char* str) {
    for (int i = 0; i < std::strlen(str); ++i) {
        lcd_byte(str[i], LCD_CHR);
    }
}

class LCDNode : public rclcpp::Node {
public:
    LCDNode() : Node("lcd_node"), lane_text_("Current Lane: - "), threat_text_("No Threat      ") {
        lane_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/lane_info", 10,
            std::bind(&LCDNode::lane_callback, this, std::placeholders::_1)
        );

        threat_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/threat", 10,
            std::bind(&LCDNode::threat_callback, this, std::placeholders::_1)
        );

        if (wiringPiSetup() == -1) {
            RCLCPP_ERROR(this->get_logger(), "WiringPi 초기화 실패");
            rclcpp::shutdown();
        }

        fd = wiringPiI2CSetup(LCD_ADDR);
        if (fd < 0) {
            RCLCPP_ERROR(this->get_logger(), "I2C LCD 연결 실패");
            rclcpp::shutdown();
        }

        lcd_init();
        update_lcd();
    }

private:
    void lane_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        int lane = msg->data;
        if (lane == 1) {
            lane_text_ = "Current Lane: 1 ";
        } else if (lane == 2) {
            lane_text_ = "Current Lane: 2 ";
        } else {
            lane_text_ = "No Lane Detected";
        }
        update_lcd();
    }

    void threat_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            threat_text_ = "!! THREAT FOUND!";
        } else {
            threat_text_ = "No Threat      ";
        }
        update_lcd();
    }

    void update_lcd() {
        lcd_loc(1);
        lcd_write(lane_text_.c_str());
        lcd_loc(2);
        lcd_write(threat_text_.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lane_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr threat_sub_;
    std::string lane_text_;
    std::string threat_text_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LCDNode>());
    rclcpp::shutdown();
    return 0;
}