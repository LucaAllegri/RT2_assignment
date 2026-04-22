#include <memory>
#include <limits>
#include <iostream>
#include <thread>
#include <atomic>
#include <unistd.h>
#include <sys/select.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

std::atomic<bool> cancel_requested{false};
std::atomic<bool> goal_running{false};
std::string goal_state = "STILL";
int pipe_fd[2]; // 0-read, 1-write

void status_callback(const std_msgs::msg::String::SharedPtr msg) {
    goal_state = msg->data;
}

void check_cancelled_request() {
    std::cout << "\nPress 'c' + ENTER to cancel the goal or wait...\n";

    while (rclcpp::ok() && goal_running) {

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);   // stdin monitoring
        FD_SET(pipe_fd[0], &fds);     // pipe monitoring

        int max_fd = std::max(STDIN_FILENO, pipe_fd[0]) + 1;

        // wait until stdin or pipes have data
        int ret = select(max_fd, &fds, nullptr, nullptr, nullptr);

        if (ret < 0) break; //error

        // main wrote into pipe, goal ended
        if (FD_ISSET(pipe_fd[0], &fds)) {
            char buf;
            read(pipe_fd[0], &buf, 1); // empty the pipe
            return;
        }

        // input on stdin
        if (FD_ISSET(STDIN_FILENO, &fds)) {
            std::string cmd;
            std::getline(std::cin, cmd);
            if (cmd == "c" || cmd == "C") {
                cancel_requested = true;
                return;
            }
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

    auto goal_pub   = node->create_publisher<std_msgs::msg::Float64MultiArray>("goal_topic", 10);
    auto cancel_pub = node->create_publisher<std_msgs::msg::Bool>("cancel_topic", 10);
    auto status_sub = node->create_subscription<std_msgs::msg::String>("goal_status", 10, status_callback);

    //CREATE PIPE
    pipe(pipe_fd);

    while (rclcpp::ok()) {

        cancel_requested = false;
        goal_running     = false;
        goal_state       = "STILL";

        double x, y, th;
        std::cout << "\n=== New Goal ===\n";
        std::cout << "Insert x target: ";
        check_input(x);
        std::cout << "Insert y target: ";
        check_input(y);
        std::cout << "Insert theta target: ";
        check_input(th);

        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        std_msgs::msg::Float64MultiArray msg;
        msg.data = {x, y, th};
        goal_pub->publish(msg);
        goal_running = true;

        RCLCPP_INFO(node->get_logger(), "Goal sent: x=%.2f y=%.2f th=%.2f", x, y, th);

        std::thread t(check_cancelled_request);

        rclcpp::Rate rate(20);
        while (rclcpp::ok() && goal_running) {

            rclcpp::spin_some(node);

            if (cancel_requested) {
                std_msgs::msg::Bool cancel_msg;
                cancel_msg.data = true;
                cancel_pub->publish(cancel_msg);
                std::cout << "\nGOAL CANCELLED\n";
                goal_running = false;
                break;
            }

            if (goal_state == "SUCCEEDED") {
                std::cout << "\nGOAL REACHED\n";
                goal_running = false;
                break;
            }

            if (goal_state == "ABORTED") {
                std::cout << "\nGOAL FAILED\n";
                goal_running = false;
                break;
            }

            rate.sleep();
        }

        //THE THREAD DIDN'T RECIVE ANYTHING YET, SO I WRITE ON THE PIPE TO UNBLOCK THE THREAD IF IT IS WAITING ON select()
        if (!cancel_requested) {
            char signal_byte = 1;
            write(pipe_fd[1], &signal_byte, 1);
        }

        t.join(); //THREAD ENDS

        std::cout << "\nPress ENTER for a new goal or 'q' + ENTER to quit...\n";
        std::string end_cmd;
        std::getline(std::cin, end_cmd);

        if (end_cmd == "q" || end_cmd == "Q") {
            std::cout << "\nExiting...\n";
            break;
        }
    }

    //CLOSE PIPE
    close(pipe_fd[0]);
    close(pipe_fd[1]);
    rclcpp::shutdown();
    return 0;
}