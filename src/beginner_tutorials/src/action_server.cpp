/*
 * @Author Youbiao He hybtalented@163.com
 * @Date 2022-07-06
 * @LastEditors Youbiao He
 * @LastEditTime 2022-07-07
 * @FilePath /src/beginner_tutorials/src/action_server.cpp
 * @Description
 *
 * @Example
 */
#include <functional>
#include <memory>
#include <thread>

#include <beginner_tutorials/action/fibonacci.hpp>
#include <beginner_tutorials/visibility_control.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

class FibonacciActionServer : public rclcpp::Node {
public:
  using Fibonacci = beginner_tutorials::action::Fibonacci;
  using FibonacciGoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;
  using FibonacciGoalHandleSharedPtr = const std::shared_ptr<FibonacciGoalHandle>;
  BEGINNER_TUTORIALS_EXPORT explicit  FibonacciActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : rclcpp::Node("fibonacci_action_server", options) {
    using namespace std::placeholders;
    _action_server = rclcpp_action::create_server<Fibonacci>(
        this, "fibonacci",
        std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
        std::bind(&FibonacciActionServer::handle_cancel, this, _1),
        std::bind(&FibonacciActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr _action_server;
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              Fibonacci::Goal::ConstSharedPtr goal) {
    RCLCPP_INFO(get_logger(), "Received goal request with order %d",
                goal->order);
    RCL_UNUSED(uuid);
    if (goal->order > 0) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    return rclcpp_action::GoalResponse::REJECT;
  }
  void handle_accepted(FibonacciGoalHandleSharedPtr goal_handle) {
    std::thread thread(std::bind(&FibonacciActionServer::execute, this, goal_handle));
    thread.detach();
  }

  rclcpp_action::CancelResponse
  handle_cancel(FibonacciGoalHandleSharedPtr goal_handle) {
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    RCL_UNUSED(goal_handle);
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void execute(FibonacciGoalHandleSharedPtr goal_handle) {
    RCLCPP_INFO(get_logger(), "Execute goal");
    rclcpp::Rate loop_rate(1);
    const Fibonacci::Goal::ConstSharedPtr goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto &sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);

    auto result = std::make_shared<Fibonacci::Result>();
    for (int i = 1; i < goal->order && rclcpp::ok(); ++i) {
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "Goal Canceled");
        return;
      }
      sequence.push_back(sequence[i] + sequence[i - 1]);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(get_logger(), "Publish feedback");

      loop_rate.sleep();
    }
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "Goal succeed");
    }
  }
};

RCLCPP_COMPONENTS_REGISTER_NODE(FibonacciActionServer);