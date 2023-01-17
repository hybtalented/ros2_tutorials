/*
 * @Author Youbiao He hybtalented@163.com
 * @Date 2022-07-07
 * @LastEditors Youbiao He
 * @LastEditTime 2022-10-26
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
    /**
     * 创建一个动作客户端。 create_client
     * 的第一个参数的为ros节点实例，客户端被添加到的节点；第二个参数为动作名称。
     * create_client 返回客户端实例，实例销毁时
     */
    client = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
    /**
     * 创建一个计时器不断发送目标
     */
    timer = create_wall_timer(
        500ms, std::bind(&FibonacciActionClient::sendGoal, this));
  }
  void sendGoal() {
    using namespace std::placeholders;
    /**
     * 停止计时器
     */
    timer->cancel();
    /**
     * 等待客户端创建
     */
    if (!client->wait_for_action_server()) {
      RCLCPP_ERROR(get_logger(), "Action server not availiable after waiting");
      rclcpp::shutdown();
      return;
    }
    /**
     * 动作目标
     */
    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(get_logger(), "Send goal");
    /**
     * 目标发送选项， 可以用于绑定动作的反馈回调，目标回调，和结果回调
     */
    auto send_goal_option = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_option.feedback_callback =
        std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    send_goal_option.goal_response_callback =
        std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
    send_goal_option.result_callback =
        std::bind(&FibonacciActionClient::result_callback, this, _1);
    /**
     * 发送动作目标。方法返回一个 std::shared_future
     * 对象，可以用于等待动作完成，并获取动作结果。
     */
    auto future = client->async_send_goal(goal_msg, send_goal_option);
  }

private:
  /**
   * 目标回调函数
   *
   * 在目标被服务器处理后调用
   * @param goal_handle 动作目标句柄
   */
  void goal_response_callback(FibonacciGoalHandleConstSharedPtr goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by the server!");
    } else {
      RCLCPP_INFO(get_logger(),
                  "Goal accepted by the server, wait for result ...");
    }
  }
  /**
   * 动作执行返回回调函数
   *
   * 在接收到服务器的返回后调用
   * @param goal_handle 动作目标句柄
   * @param feedback 反馈内容
   */
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
  /**
   * 动作目标完成回调函数
   *
   * 在动作目标执行完成后，客户端将会接受到目标执行结果
   * @param result 动作执行结果
   */
  void result_callback(const FibonacciGoalHandle::WrappedResult &result) {
    using rclcpp_action::ResultCode;
    /**
     * 可能有3总运行结果， SUCCEEDED 成功完成， ABORTED 服务端取消，
     * CANCELED客户端取消
     */
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
    // 在动作执行完成后调用 shutdown ， ros2 节点退出
    rclcpp::shutdown();
  }
  rclcpp_action::Client<Fibonacci>::SharedPtr client;
  rclcpp::TimerBase::SharedPtr timer;
};
/**
 * 将节点注册为一个ROS组件
 */
RCLCPP_COMPONENTS_REGISTER_NODE(FibonacciActionClient);