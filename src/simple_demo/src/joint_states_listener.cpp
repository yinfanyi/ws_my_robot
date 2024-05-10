#include <iostream>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // 创建 socket
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    // char buffer[1024] = {0};

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(9090);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }

    if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
        perror("accept");
        exit(EXIT_FAILURE);
    }

    // 发送接收到的消息给客户端
    std::string jointStateMsg = "Joint State: ";
    for (size_t i = 0; i < msg->name.size(); ++i) {
        jointStateMsg += msg->name[i] + ": " + std::to_string(msg->position[i]) + ", ";
    }

    send(new_socket, jointStateMsg.c_str(), jointStateMsg.length(), 0);
    std::cout << "Sent joint state message to client" << std::endl;

    close(new_socket);
    close(server_fd);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("joint_states_listener");
    auto subscription = node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, jointStateCallback);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
