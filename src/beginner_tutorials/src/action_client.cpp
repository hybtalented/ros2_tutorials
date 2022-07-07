/*
 * @Author Youbiao He hybtalented@163.com
 * @Date 2022-07-07
 * @LastEditors Youbiao He
 * @LastEditTime 2022-07-07
 * @FilePath /src/beginner_tutorials/src/action_client.cpp
 * @Description
 *
 * @Example
 */
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include <beginner_tutorials/action/fibonacci.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;
class FibonacciActionClient : public rclcpp::Node {
public:
  using Fibonacci = beginner_tutorials::action::Fibonacci;
  using FibonacciGoalHandle = rclcpp_action::ClientGoalHandle<Fibonacci>;
  using FibonacciGoalHandleConstSharedPtr =
      const FibonacciGoalHandle::SharedPtr &;
  explicit FibonacciActionClient(const rclcpp::NodeOptions &options)
      : rclcpp::Node("fibonacci_action_client", options) {
    client = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
    timer = create_wall_timer(
        500ms, std::bind(&FibonacciActionClient::sendGoal, this));
  }
  void sendGoal() {
    using namespace std::placeholders;
    timer->cancel();
    if (!client->wait_for_action_server()) {
      RCLCPP_ERROR(get_logger(), "Action server not availiable after waiting");
      rclcpp::shutdown();
      return;
    }
    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(get_logger(), "Send goal");
    auto send_goal_option = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_option.feedback_callback =
        std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    send_goal_option.goal_response_callback =
        std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
    send_goal_option.result_callback =
        std::bind(&FibonacciActionClient::result_callback, this, _1);
    auto future = client->async_send_goal(goal_msg, send_goal_option);
  }

private:
  void goal_response_callback(FibonacciGoalHandleConstSharedPtr goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by the server!");
    } else {
      RCLCPP_INFO(get_logger(),
                  "Goal accepted by the server, wait for result ...");
    }
  }
  void feedback_callback(FibonacciGoalHandleConstSharedPtr goal_handle,
                         const Fibonacci::Feedback::ConstSharedPtr feedback) {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(get_logger(), ss.str().c_str());
    RCL_UNUSED(goal_handle);
  }
  void result_callback(const FibonacciGoalHandle::WrappedResult &result) {
    using rclcpp_action::ResultCode;
    switch (result.code) {
    case ResultCode::SUCCEEDED:
      break;
    case ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "goal was aborted");
      break;
    case ResultCode::CANCELED:
      RCLCPP_ERROR(get_logger(), "goal was canceled");
      break;
    default:
      RCLCPP_ERROR(get_logger(), "unknow result");
      break;
    }
    std::stringstream ss;
    ss << "result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
  rclcpp_action::Client<Fibonacci>::SharedPtr client;
  rclcpp::TimerBase::SharedPtr timer;
};

RCLCPP_COMPONENTS_REGISTER_NODE(FibonacciActionClient);