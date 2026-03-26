#include "robotiq_controllers/gripper_action_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace robotiq_controllers
{

// ===========================================================================
//                      SINGLE GRIPPER CONTROLLER
// ===========================================================================

SingleGripperController::SingleGripperController(rclcpp::Node* node, const GripperConfig& config)
    : node_(node), cfg_(config), logger_(node->get_logger())
{
    auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    cmd_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(config.command_topic, 10);
    holding_pub_ = node->create_publisher<std_msgs::msg::Bool>(config.holding_topic, 10);

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cb_group;

    state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
        config.state_topic, 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { joint_state_callback(msg); },
        sub_opts);

    action_server_ = rclcpp_action::create_server<GripperCommand>(
        node, config.action_topic,
        [this](const rclcpp_action::GoalUUID& uuid,
               std::shared_ptr<const GripperCommand::Goal> goal) {
            return handle_goal(uuid, goal);
        },
        [this](std::shared_ptr<GoalHandle> gh) { return handle_cancel(gh); },
        [this](std::shared_ptr<GoalHandle> gh) { handle_accepted(gh); },
        rcl_action_server_get_default_options(), cb_group);

    RCLCPP_INFO(logger_, "[%s] Action server on '%s'\n  Commands -> '%s'\n  States  <- '%s'",
                config.name.c_str(), config.action_topic.c_str(), config.command_topic.c_str(),
                config.state_topic.c_str());
}

void SingleGripperController::joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::vector<double> positions;
    for (const auto& jn : cfg_.joint_names) {
        auto it = std::find(msg->name.begin(), msg->name.end(), jn);
        if (it != msg->name.end()) {
            auto idx = std::distance(msg->name.begin(), it);
            positions.push_back(msg->position[idx]);
        }
    }

    if (positions.empty()) {
        if (!msg->position.empty()) {
            positions = std::vector<double>(msg->position.begin(), msg->position.end());
        } else {
            return;
        }
    }

    double avg_pos = 0.0;
    for (double p : positions) {
        avg_pos += p;
    }
    avg_pos /= static_cast<double>(positions.size());

    double now = node_->now().seconds();

    std::lock_guard<std::mutex> lock(mutex_);
    current_position_ = avg_pos;
    state_received_ = true;

    if (!goal_active_) {
        return;
    }

    // Track position history
    position_history_.push_back({now, avg_pos});
    while (position_history_.size() > kMaxHistorySize) {
        position_history_.pop_front();
    }

    // Check reached_goal
    if (std::abs(avg_pos - target_position_) <= cfg_.position_tolerance) {
        reached_goal_ = true;
        stalled_ = false;
        holding_ = false;
        return;
    }

    // Stall detection
    if (position_history_.size() >= 2) {
        auto [t0, p0] = position_history_.front();
        auto [t1, p1] = position_history_.back();
        double elapsed = t1 - t0;
        if (elapsed > 0.05) {
            double velocity = std::abs(p1 - p0) / elapsed;
            if (velocity < cfg_.stall_velocity_threshold) {
                if (stall_start_time_ < 0.0) {
                    stall_start_time_ = now;
                } else if ((now - stall_start_time_) >= cfg_.stall_time) {
                    stalled_ = true;
                    holding_ = true;
                }
            } else {
                stall_start_time_ = -1.0;
            }
        }
    }
}

rclcpp_action::GoalResponse SingleGripperController::handle_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const GripperCommand::Goal> goal)
{
    RCLCPP_INFO(logger_, "[%s] Goal: pos=%.4f, effort=%.1f", cfg_.name.c_str(),
                goal->command.position, goal->command.max_effort);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SingleGripperController::handle_cancel(
    std::shared_ptr<GoalHandle> /*goal_handle*/)
{
    RCLCPP_INFO(logger_, "[%s] Cancel requested", cfg_.name.c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SingleGripperController::handle_accepted(std::shared_ptr<GoalHandle> goal_handle)
{
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
}

void SingleGripperController::execute(std::shared_ptr<GoalHandle> goal_handle)
{
    double target_pos = goal_handle->get_goal()->command.position;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        target_position_ = target_pos;
        goal_active_ = true;
        stalled_ = false;
        reached_goal_ = false;
        holding_ = false;
        stall_start_time_ = -1.0;
        position_history_.clear();
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!state_received_) {
            RCLCPP_WARN(logger_,
                        "[%s] No joint state received yet from '%s'. "
                        "Make sure Isaac Sim is publishing gripper joint states.",
                        cfg_.name.c_str(), cfg_.state_topic.c_str());
        }
    }

    auto feedback = std::make_shared<GripperCommand::Feedback>();
    const auto feedback_period =
        std::chrono::duration<double>(1.0 / cfg_.feedback_rate);
    constexpr double kTimeout = 10.0;
    auto start_time = node_->now();

    while (rclcpp::ok()) {
        // Publish command to Isaac Sim
        auto cmd_msg = std_msgs::msg::Float64MultiArray();
        cmd_msg.data.assign(cfg_.joint_names.size(), target_pos);
        cmd_pub_->publish(cmd_msg);

        // Check cancellation
        if (goal_handle->is_canceling()) {
            auto result = std::make_shared<GripperCommand::Result>();
            {
                std::lock_guard<std::mutex> lock(mutex_);
                result->position = current_position_;
                result->stalled = stalled_;
                result->reached_goal = reached_goal_;
                goal_active_ = false;
            }
            goal_handle->canceled(result);
            RCLCPP_INFO(logger_, "[%s] Goal canceled", cfg_.name.c_str());
            return;
        }

        // Read state
        double cur_pos;
        bool stalled, reached;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cur_pos = current_position_;
            stalled = stalled_;
            reached = reached_goal_;
        }

        // Publish feedback
        feedback->position = cur_pos;
        feedback->stalled = stalled;
        feedback->reached_goal = reached;
        goal_handle->publish_feedback(feedback);

        // Terminal conditions
        if (reached || stalled) {
            break;
        }

        // Safety timeout
        if ((node_->now() - start_time).seconds() > kTimeout) {
            RCLCPP_WARN(logger_, "[%s] Goal timed out after %.0fs", cfg_.name.c_str(), kTimeout);
            std::lock_guard<std::mutex> lock(mutex_);
            stalled_ = true;
            break;
        }

        std::this_thread::sleep_for(
            std::chrono::duration_cast<std::chrono::nanoseconds>(feedback_period));
    }

    // Build result
    auto result = std::make_shared<GripperCommand::Result>();
    std::string status;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        result->position = current_position_;
        result->stalled = stalled_;
        result->reached_goal = reached_goal_;
        goal_active_ = false;

        status = result->reached_goal    ? "reached goal"
                 : result->stalled       ? "stalled (holding)"
                                         : "unknown";
    }

    RCLCPP_INFO(logger_, "[%s] Done: %s, pos=%.4f", cfg_.name.c_str(), status.c_str(),
                result->position);

    goal_handle->succeed(result);
}

void SingleGripperController::publish_holding()
{
    auto msg = std_msgs::msg::Bool();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        msg.data = holding_;
    }
    holding_pub_->publish(msg);
}

// ===========================================================================
//                          MAIN NODE
// ===========================================================================

GripperActionServerNode::GripperActionServerNode(const rclcpp::NodeOptions& options)
    : Node("gripper_action_server", options)
{
    // Declare parameters
    declare_parameter("left_action_topic", "/l_arm_robotiq_gripper_controller/gripper_cmd");
    declare_parameter("right_action_topic", "/r_arm_robotiq_gripper_controller/gripper_cmd");
    declare_parameter("left_command_topic", "/l_gripper/joint_command");
    declare_parameter("right_command_topic", "/r_gripper/joint_command");
    declare_parameter("left_state_topic", "/l_gripper/joint_state");
    declare_parameter("right_state_topic", "/r_gripper/joint_state");
    declare_parameter("left_holding_topic", "/l_arm_gripper/holding");
    declare_parameter("right_holding_topic", "/r_arm_gripper/holding");
    declare_parameter("position_tolerance", 0.005);
    declare_parameter("stall_velocity_threshold", 0.0005);
    declare_parameter("stall_time", 0.3);
    declare_parameter("holding_publish_rate", 10.0);

    GripperConfig left_cfg;
    left_cfg.name = "left_gripper";
    left_cfg.action_topic = get_parameter("left_action_topic").as_string();
    left_cfg.command_topic = get_parameter("left_command_topic").as_string();
    left_cfg.state_topic = get_parameter("left_state_topic").as_string();
    left_cfg.holding_topic = get_parameter("left_holding_topic").as_string();
    left_cfg.joint_names = {"l_arm_finger_joint"};
    left_cfg.position_tolerance = get_parameter("position_tolerance").as_double();
    left_cfg.stall_velocity_threshold = get_parameter("stall_velocity_threshold").as_double();
    left_cfg.stall_time = get_parameter("stall_time").as_double();

    GripperConfig right_cfg;
    right_cfg.name = "right_gripper";
    right_cfg.action_topic = get_parameter("right_action_topic").as_string();
    right_cfg.command_topic = get_parameter("right_command_topic").as_string();
    right_cfg.state_topic = get_parameter("right_state_topic").as_string();
    right_cfg.holding_topic = get_parameter("right_holding_topic").as_string();
    right_cfg.joint_names = {"r_arm_finger_joint"};
    right_cfg.position_tolerance = get_parameter("position_tolerance").as_double();
    right_cfg.stall_velocity_threshold = get_parameter("stall_velocity_threshold").as_double();
    right_cfg.stall_time = get_parameter("stall_time").as_double();

    double holding_rate = get_parameter("holding_publish_rate").as_double();

    left_gripper_ = std::make_unique<SingleGripperController>(this, left_cfg);
    right_gripper_ = std::make_unique<SingleGripperController>(this, right_cfg);

    holding_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / holding_rate)),
        [this]() { holding_timer_callback(); });

    RCLCPP_INFO(get_logger(), "Gripper action server node ready.");
}

void GripperActionServerNode::holding_timer_callback()
{
    left_gripper_->publish_holding();
    right_gripper_->publish_holding();
}

}  // namespace robotiq_controllers

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<robotiq_controllers::GripperActionServerNode>();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
