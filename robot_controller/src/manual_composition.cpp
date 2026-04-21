#include <memory>
#include <limits>
#include <iostream>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

//STATE 
std::atomic<bool> cancel_requested{false};
std::atomic<bool> goal_running{false};

std::string goal_state = "STILL";

void status_callback(const std_msgs::msg::String::SharedPtr msg) {
    goal_state = msg->data;
}

void check_cancelled_request(){
    std::string cmd;

    while (rclcpp::ok() && goal_running) {

        std::cout << "\nPress 'c' + ENTER to cancel the goal or wait...\n";
        std::cin >> cmd;

        if (cmd == "c") {
            cancel_requested = true;
            return;
        }
    }
}

void check_input(double &i) {
    while (!(std::cin >> i)) {
        std::cout << "Input invalid. Insert a number: ";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
}

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("user_interface");

    auto goal_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("goal_topic", 10);
    auto cancel_pub = node->create_publisher<std_msgs::msg::Bool>("cancel_topic", 10);

    auto status_sub = node->create_subscription<std_msgs::msg::String>("goal_status", 10, status_callback);

    double x, y, th;

    std::cout << "Insert x target: ";
    check_input(x);

    std::cout << "Insert y target: ";
    check_input(y);

    std::cout << "Insert theta target: ";
    check_input(th);

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {x, y, th};
    goal_pub->publish(msg);

    goal_running = true;

    RCLCPP_INFO(node->get_logger(),"Goal sent: x=%.2f y=%.2f th=%.2f",x, y, th);

    std::thread t(check_cancelled_request);
    t.detach();

    rclcpp::Rate rate(20);

    while (rclcpp::ok() && goal_running) {

        rclcpp::spin_some(node);

        // CANCEL
        if (cancel_requested) {
            std_msgs::msg::Bool cancel_msg;
            cancel_msg.data = true;
            cancel_pub->publish(cancel_msg);

            std::cout << "\nGOAL CANCELLED\n";
            goal_running = false;
            break;
        }

        // SUCCESSO
        if (goal_state == "SUCCEEDED") {
            std::cout << "\nGOAL REACHED\n";
            goal_running = false;
            break;
        }

        // FALLIMENTO (opzionale)
        if (goal_state == "ABORTED") {
            std::cout << "\nGOAL FAILED\n";
            goal_running = false;
            break;
        }

        rate.sleep();
    }

    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cin.get();
}