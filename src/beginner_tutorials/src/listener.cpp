/*
 * @Author Youbiao He hybtalented@163.com
 * @Date 2022-06-29
 * @LastEditors Youbiao He
 * @LastEditTime 2022-06-29
 * @FilePath /src/beginner_tutorials/src/listener.cpp
 * @Description
 *
 * @Example
 */
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char **argv) {
  /**
   * 初始化ros节点
   */
  rclcpp::init(argc, argv);

  /**
   * 创建一个节点, 实现与ros系统的交互
   */
  rclcpp::Node::SharedPtr node(new rclcpp::Node("listener"));

  /**
   * 订阅对应的 ros 主题
   *
   * 通知 ros master 节点该节点订阅的相应的主题, 并自动连接到对应主题的发布者上.
   * 当发布者发布该主题的消息 后, 将会调用回调函数 chatterCallback. subscribe
   * 方法返回一个订阅者实例, 节点将会一直订阅该主题,
   * 直到该订阅者实例及其所有的拷贝被销毁.
   *
   * subscribe 的第二个参数表示消息接收队列的大小.
   * 当消息的接收速度大于处理的速度时, 接收到的消息将会缓冲到 消息接收队列中,
   * 当消息接收队列满了以后, 后续的消息将会被丢弃.
   */
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub =
      node->create_subscription<std_msgs::msg::String>(
          "chatter", 10, [&](const std_msgs::msg::String &msg) {
            RCLCPP_INFO(node->get_logger(), "I hear: [%s]", msg.data.c_str());
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