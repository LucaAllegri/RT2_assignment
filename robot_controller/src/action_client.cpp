#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "action_interfaces/action/target.hpp"
#include "message_custom/msg/goal_frame.hpp"

class RobotActionClient : public rclcpp::Node{
    public:
        using Target = action_interfaces::action::Target;
        using GoalHandleTarget = rclcpp_action::ClientGoalHandle<Target>;

        RobotActionClient() : Node("robot_action_client"), cancel_sent_(false){
            action_client_ = rclcpp_action::create_client<Target>(this, "target");
        }

        goal_frame_broad_pub_ = this->create_publisher<message_custom::msg::GoalFrame>("/goal_frame", 10);

        void send_goal(double x_goal, double y_goal, double theta_goal){
            if (!action_client_->wait_for_action_server()) {
                RCLCPP_ERROR(this->get_logger(), "Action server not available");
                return;
            }

            cancel_sent_ = false;

            auto goal_msg = Target::Goal();
            goal_msg.goal_pose = {x_goal, y_goal, theta_goal};

            message_custom::msg::GoalFrame frame_to_broad;
            frame_to_broad.name_frame = "GoalFrame";
            frame_to_broad.x_goal = x_goal;
            frame_to_broad.y_goal = y_goal;
            frame_to_broad.theta_goal = theta_goal;

            goal_frame_broad_pub_ -> publish(frame_to_broad)

            using namespace std::placeholders;

            rclcpp_action::Client<Target>::SendGoalOptions options;

            options.goal_response_callback = std::bind(&RobotActionClient::goal_response_callback, this, _1);

            options.feedback_callback =std::bind(&RobotActionClient::feedback_callback, this, _1, _2);

            options.result_callback = std::bind(&RobotActionClient::result_callback, this, _1);

            action_client_->async_send_goal(goal_msg, options); 
        }

    private:

        void goal_response_callback(GoalHandleTarget::SharedPtr goal_handle){
            if (!goal_handle) {
                RCLCPP_INFO(this->get_logger(), "Goal rejected");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Goal accepted");
            current_goal_handle_ = goal_handle;
        }

        void feedback_callback(GoalHandleTarget::SharedPtr,const std::shared_ptr<const Target::Feedback> feedback){
            double remaining = feedback->current_pose[0];

            RCLCPP_INFO(this->get_logger(),"Feedback: remaining angle = %f",remaining);

            if (cancel_sent_ || !current_goal_handle_) {
                return;
            }

            if (remaining > 1.5) {
                cancel_sent_ = true;
                RCLCPP_WARN(this->get_logger(),
                            "Remaining angle less than 1.0, cancelling goal...");

                action_client_->async_cancel_goal(current_goal_handle_);
            }

        }

        void result_callback(const GoalHandleTarget::WrappedResult & result){
            RCLCPP_INFO(this->get_logger(),"Result status=%d, delta=%f",result.code,result.result->final_pose[0]);

            //rclcpp::shutdown();
        }

        //PUBLISHER
        rclcpp::Publisher<message_custom::msg::GoalFrame>::SharedPtr goal_frame_broad_pub_;

        //CLIENT
        rclcpp_action::Client<Target>::SharedPtr action_client_;

        //VARIABLES
        GoalHandleTarget::SharedPtr current_goal_handle_;
        bool cancel_sent_;
        double input;

};

int main(int argc, char ** argv){
    rclcpp::init(argc, argv);

    // Crea il nodo client
    auto node = std::make_shared<RobotActionClient>();

    double x_target, y_target, theta_target;

    std::cout << "insert x target: ";
    if (!(std::cin >> x_target)) {
        std::cout << "Invalid input.\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    std::cout << "insert y target: ";
    if (!(std::cin >> y_target)) {
        std::cout << "Invalid input.\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    std::cout << "insert theta target: ";
    if (!(std::cin >> theta_target)) {
        std::cout << "Invalid input.\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    node->send_goal(x_target,y_target,theta_target);

    // Spin per qualche tempo per inviare goal e ricevere feedback
    rclcpp::spin(node);

    //rclcpp::shutdown();
    return 0;
}



