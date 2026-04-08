#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_interfaces/action/target.hpp"

#include "message_custom/msg/goal_frame.hpp"


namespace robot_controller{

    class RobotActionClient : public rclcpp::Node{
        public:
            using Target = action_interfaces::action::Target;
            using GoalHandleTarget = rclcpp_action::ClientGoalHandle<Target>;

            explicit RobotActionClient(const rclcpp::NodeOptions & options) : Node("robot_action_client", options), cancel_sent_(false){

                action_client_ = rclcpp_action::create_client<Target>(this, "target");

                goal_frame_broad_pub_ = this->create_publisher<message_custom::msg::GoalFrame>("/goal_frame", 10);
            }

            void send_goal(double x_goal, double y_goal, double theta_goal){
                if (!action_client_->wait_for_action_server()) {
                    RCLCPP_ERROR(this->get_logger(), "Action server not available");
                    return;
                }

                cancel_sent_ = false;

                auto goal_msg = Target::Goal();
                goal_msg.goal_pose = {x_goal, y_goal, theta_goal};

                message_custom::msg::GoalFrame frame_to_broad;
                frame_to_broad.x_goal = x_goal;
                frame_to_broad.y_goal = y_goal;
                frame_to_broad.theta_goal = theta_goal;

                goal_frame_broad_pub_-> publish(frame_to_broad);

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
                double remaining_x = feedback->current_pose[0];
                double remaining_y = feedback->current_pose[1];

                RCLCPP_INFO(this->get_logger(),"Feedback: remaining x = %f, remaining y = %f",remaining_x, remaining_y);
            }

            void result_callback(const GoalHandleTarget::WrappedResult & result){

                RCLCPP_INFO(this->get_logger(),"Result status=%d, x_=%f, y_= %f",result.code,result.result->final_pose[0],result.result->final_pose[1]);

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
}

RCLCPP_COMPONENTS_REGISTER_NODE(robot_controller::RobotActionClient)

/*int main(int argc, char ** argv){
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
}*/



