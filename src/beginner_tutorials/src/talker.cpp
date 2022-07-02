/*
 * @Author Youbiao He hybtalented@163.com
 * @Date 2022-06-29
 * @LastEditors Youbiao He
 * @LastEditTime 2022-07-02
 * @FilePath /src/beginner_tutorials/src/talker.cpp
 * @Description
 *
 * @Example
 */
#include <chrono>
#include <functional>
#include <memory.h>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
int main(int argc, char **argv) {
  /**
   * 初始化 ros 节点
   */
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node(new rclcpp::Node("talker"));
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher =
      node->create_publisher<std_msgs::msg::String>("chatter", 10);
  node->declare_parameter("target", "world");
  int count = 0;
  /**
   * 创建一个定时器
   */
  rclcpp::TimerBase::SharedPtr timer = node->create_wall_timer(500ms, [&]() {
    std_msgs::msg::String msg;
    rclcpp::Parameter target = node->get_parameter("target");
    msg.data =
        "hello " + target.get_value<std::string>() + std::to_string(count++);
    RCLCPP_INFO(node->get_logger(), "Publishing '%s'", msg.data.c_str());
    /**
     * 发布一个消息, 发布的消息类型必须要与 advertise 创建是指定的模板参数一致.
     */
    publisher->publish(msg);
  });
  /**
   * 进入ros的消息循环
   *
   * ros 消息循环将会处理各种消息回调, 当没有任何回调消息循环将会阻塞, 因此不会
   * 大量占用cpu. 消息循环会一直运行直到
   * 1. 收到 SIGINT 信号(如用户按下了 Ctrl+C)
   * 2. 应用程序调用了 ros::shutdown 方法
   * 3. 所有的 NodeHandle 实例都被销毁了
   * 4. 另一个同名的节点启动了
   *
   * 需要注意的是它不会处理自定义队列中的回调.
   */
  rclcpp::spin(node);
  /**
   * 关闭 rcl cpp 上下文
   */
  rclcpp::shutdown();
  return 0;
}