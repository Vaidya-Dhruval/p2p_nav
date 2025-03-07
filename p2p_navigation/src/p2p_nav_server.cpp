#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <p2p_navigation_interfaces/action/navigate_to_pose.hpp>

class P2PNavigationActionServer : public rclcpp::Node
{
public:
    using P2PNavigateToPose = p2p_navigation_interfaces::action::NavigateToPose;
    using Nav2NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<P2PNavigateToPose>;
    using Nav2GoalHandle = rclcpp_action::ClientGoalHandle<Nav2NavigateToPose>;

    P2PNavigationActionServer()
        : Node("p2p_navigation_action_server")
    {
        // Create action server
        this->action_server_ = rclcpp_action::create_server<P2PNavigateToPose>(
            this,
            "p2p_navigation",
            std::bind(&P2PNavigationActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&P2PNavigationActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&P2PNavigationActionServer::handle_accepted, this, std::placeholders::_1));

        // Create Nav2 action client
        this->nav2_action_client_ = rclcpp_action::create_client<Nav2NavigateToPose>(this, "navigate_to_pose");

        RCLCPP_INFO(this->get_logger(), "P2P Navigation Action Server Started");
    }

private:
    rclcpp_action::Server<P2PNavigateToPose>::SharedPtr action_server_;
    rclcpp_action::Client<Nav2NavigateToPose>::SharedPtr nav2_action_client_;
    Nav2GoalHandle::SharedPtr nav2_goal_handle_; // Store goal handle

    // Goal callback
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const P2PNavigateToPose::Goal> goal)
    {
        (void)goal; // Explicitly ignore the 'goal' parameter
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Cancel callback
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
    {
        (void)goal_handle; // Explicitly ignore the 'goal_handle' parameter
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

        if (nav2_goal_handle_) { // Check if we have an active goal
            nav2_action_client_->async_cancel_goal(nav2_goal_handle_);
        }

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Goal accepted callback
    void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Goal accepted, sending to Nav2");

        auto goal_msg = Nav2NavigateToPose::Goal();
        goal_msg.pose = goal_handle->get_goal()->pose;

        auto send_goal_options = rclcpp_action::Client<Nav2NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&P2PNavigationActionServer::nav2_goal_response_callback, this, std::placeholders::_1, goal_handle);
        send_goal_options.feedback_callback =
            std::bind(&P2PNavigationActionServer::nav2_feedback_callback, this, std::placeholders::_1, std::placeholders::_2, goal_handle);
        send_goal_options.result_callback =
            std::bind(&P2PNavigationActionServer::nav2_result_callback, this, std::placeholders::_1, goal_handle);

        nav2_action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    // Nav2 goal response callback
    void nav2_goal_response_callback(
        Nav2GoalHandle::SharedPtr goal_handle,
        std::shared_ptr<GoalHandleNavigateToPose> p2p_goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 rejected the goal");
            p2p_goal_handle->abort(std::make_shared<P2PNavigateToPose::Result>());
        } else {
            nav2_goal_handle_ = goal_handle; // Store goal handle
        }
    }

    // Nav2 feedback callback
    void nav2_feedback_callback(
        Nav2GoalHandle::SharedPtr,
        std::shared_ptr<const Nav2NavigateToPose::Feedback> feedback,
        std::shared_ptr<GoalHandleNavigateToPose> p2p_goal_handle)
    {
        auto p2p_feedback = std::make_shared<P2PNavigateToPose::Feedback>();
        p2p_feedback->current_pose = feedback->current_pose;
        p2p_feedback->distance_remaining = feedback->distance_remaining;

        // Convert duration to float seconds
        p2p_feedback->navigation_time = feedback->navigation_time.sec +
                                        feedback->navigation_time.nanosec * 1e-9;

        p2p_goal_handle->publish_feedback(p2p_feedback);
    }

    // Nav2 result callback
    void nav2_result_callback(
        const Nav2GoalHandle::WrappedResult &result,
        std::shared_ptr<GoalHandleNavigateToPose> p2p_goal_handle)
    {
        auto p2p_result = std::make_shared<P2PNavigateToPose::Result>();

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Goal reached successfully");
            p2p_goal_handle->succeed(p2p_result);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Navigation failed");
            p2p_goal_handle->abort(p2p_result);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<P2PNavigationActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
