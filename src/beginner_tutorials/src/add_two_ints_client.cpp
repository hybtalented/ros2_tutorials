/*
 * @Author Youbiao He hybtalented@163.com
 * @Date 2022-06-30
 * @LastEditors Youbiao He
 * @LastEditTime 2022-07-07
 * @FilePath /src/beginner_tutorials/src/add_two_ints_client.cpp
 * @Description
 *
 * @Example
 */
#include <chrono>
#include <cstdlib>
#include <memory>

#include <beginner_tutorials/srv/add_two_ints.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  /**
   * 初始化 ros 节点
   */
  rclcpp::init(argc, argv);
  /**
   * 创建 ros 节点句柄
   */
  rclcpp::Node::SharedPtr node =
      rclcpp::Node::make_shared("add_two_ints_client");

  /**
   * 创建 ros 客户端
   *
   * 创建一个客户端用于调用 add_two_ints 服务,
   * 可以指定第二个参数以提高服务的调用效率, 以及第三个参数
   * 指定请求连接的连接握手时的请求头.
   *
   * serviceClient 方法返回一个服务调用客户端, 通过该客户端可以调用服务.
   */
  rclcpp::Client<beginner_tutorials::srv::AddTwoInts>::SharedPtr client =
      node->create_client<beginner_tutorials::srv::AddTwoInts>("add_two_ints");
  /**
   * 创建一个服务对象, 用于传递请求以及获取返回值
   */
  beginner_tutorials::srv::AddTwoInts::Request::SharedPtr request =
      std::make_shared<beginner_tutorials::srv::AddTwoInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(),
                   "Interupt while waiting for service. Exiting.");
      return -1;
    }
    RCLCPP_INFO(node->get_logger(), "Service %s not availiable. Wait again",
                client->get_service_name());
  }
  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    // 服务调用成功
    RCLCPP_INFO(node->get_logger(), "sum is %ld", result.get()->sum);
    return 0;
  } else {
    // 服务调用失败
    RCLCPP_INFO(node->get_logger(), "Failed to call service add_two_ints");
    return -1;
  }
}