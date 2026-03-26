#pragma once

#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/gripper_command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace robotiq_controllers
{

struct GripperConfig
{
    std::string name;
    std::string action_topic;
    std::string command_topic;
    std::string state_topic;
    std::string holding_topic;
    std::vector<std::string> joint_names;

    double position_tolerance = 0.005;
    double stall_velocity_threshold = 0.0005;
    double stall_time = 0.3;
    double feedback_rate = 30.0;
};

class SingleGripperController
{
public:
    using GripperCommand = control_msgs::action::GripperCommand;
    using GoalHandle = rclcpp_action::ServerGoalHandle<GripperCommand>;

    SingleGripperController(rclcpp::Node* node, const GripperConfig& config);

    void publish_holding();

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const GripperCommand::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle> goal_handle);

    void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);

    void execute(std::shared_ptr<GoalHandle> goal_handle);

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::Node* node_;
    GripperConfig cfg_;
    rclcpp::Logger logger_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr holding_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
    rclcpp_action::Server<GripperCommand>::SharedPtr action_server_;

    std::mutex mutex_;
    bool goal_active_ = false;
    double target_position_ = 0.0;
    double current_position_ = 0.0;
    bool state_received_ = false;

    std::deque<std::pair<double, double>> position_history_;
    static constexpr std::size_t kMaxHistorySize = 120;
    double stall_start_time_ = -1.0;
    bool stalled_ = false;
    bool reached_goal_ = false;
    bool holding_ = false;
};

class GripperActionServerNode : public rclcpp::Node
{
public:
    GripperActionServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void holding_timer_callback();

    std::unique_ptr<SingleGripperController> left_gripper_;
    std::unique_ptr<SingleGripperController> right_gripper_;
    rclcpp::TimerBase::SharedPtr holding_timer_;
};

}  // namespace robotiq_controllers
