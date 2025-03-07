#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class P2PNavigationClient : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    explicit P2PNavigationClient() : Node("p2p_navigation_client")
    {
        this->action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        if (!this->action_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }
        
        send_goal(2.0, 3.0);
    }

    void send_goal(double x, double y)
    {
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "Sending goal: (%.2f, %.2f)", x, y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&P2PNavigationClient::goal_response_callback, this, std::placeholders::_1);

        this->action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

    void goal_response_callback(const GoalHandleNavigateToPose::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Navigation failed");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<P2PNavigationClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
