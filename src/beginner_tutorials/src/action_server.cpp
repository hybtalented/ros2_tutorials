/*
 * @Author Youbiao He hybtalented@163.com
 * @Date 2022-07-06
 * @LastEditors Youbiao He
 * @LastEditTime 2022-07-15
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
    /**
     * 创建一个动作服务；第一个参数是动作服务所在的节点，第二个参数为动作名，第三、四、五个参数分别为目标处理函数，动作取消函数和动作处理函数
     */
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
      // 接受并且执行目标
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    // 拒绝执行目标
    return rclcpp_action::GoalResponse::REJECT;
  }
  void handle_accepted(FibonacciGoalHandleSharedPtr goal_handle) {
    // 处理动作，这个函数需要立即返回，否则会阻塞 ros 消息循环，因此我们创建一个线程去执行动作
    std::thread thread(std::bind(&FibonacciActionServer::execute, this, goal_handle));
    thread.detach();
  }

  rclcpp_action::CancelResponse
  handle_cancel(FibonacciGoalHandleSharedPtr goal_handle) {
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    RCL_UNUSED(goal_handle);
    // 服务器以及取消了对应的动作
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void execute(FibonacciGoalHandleSharedPtr goal_handle) {
    RCLCPP_INFO(get_logger(), "Execute goal");
    // 创建一个定时器，频率为 1Hz
    rclcpp::Rate loop_rate(1);
    const Fibonacci::Goal::ConstSharedPtr goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    // 初始化斐波那契序列
    auto &sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);

    auto result = std::make_shared<Fibonacci::Result>();
    /**
     * 迭代计算序列，如果对应的ros 上下文以及退出（即调用了 shutdown)， 循环也将会将会结束
     */
    for (int i = 1; i < goal->order && rclcpp::ok(); ++i) {
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        // 通知客户端动作已被取消
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "Goal Canceled");
        return;
      }
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // 发布反馈
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(get_logger(), "Publish feedback");
      // 等待一秒后继续迭代
      loop_rate.sleep();
    }
    if (rclcpp::ok()) {
      result->sequence = sequence;
      // 通知客户端动作以及成功完成
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "Goal succeed");
    }
  }
};
// 将节点注册为一个ROS组件
RCLCPP_COMPONENTS_REGISTER_NODE(FibonacciActionServer);