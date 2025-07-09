#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <thread>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

void threat_listener_loop(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub) {
    int server_fd, client_fd;
    struct sockaddr_in serv_addr, cli_addr;
    socklen_t cli_len = sizeof(cli_addr);

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(9999);
    bind(server_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
    listen(server_fd, 5);

    while (rclcpp::ok()) {
        client_fd = accept(server_fd, (struct sockaddr*)&cli_addr, &cli_len);
        if (client_fd < 0) continue;

        char buffer[1024] = {0};
        int bytes = read(client_fd, buffer, sizeof(buffer));
        close(client_fd);

        if (bytes > 0 && std::string(buffer).find("1") != std::string::npos) {
            auto msg = std_msgs::msg::Bool();
            msg.data = true;
            pub->publish(msg);
            RCLCPP_WARN(rclcpp::get_logger("receive_threat"), "⚠️ 위협 신호 수신!");
        }
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("receive_threat");
    auto publisher = node->create_publisher<std_msgs::msg::Bool>("/threat", 10);

    std::thread listener_thread(threat_listener_loop, publisher);
    rclcpp::spin(node);
    listener_thread.join();

    rclcpp::shutdown();
    return 0;
}