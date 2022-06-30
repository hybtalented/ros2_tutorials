/*
 * @Author Youbiao He hybtalented@163.com
 * @Date 2022-06-30
 * @LastEditors Youbiao He
 * @LastEditTime 2022-07-01
 * @FilePath /src/beginner_tutorials/src/add_two_ints_server.cpp
 * @Description
 *
 * @Example
 */
#include <functional>
#include <memory>

#include <beginner_tutorials/srv/add_two_ints.hpp>
#include <rclcpp/rclcpp.hpp>
/**
 * @brief 服务处理函数, sum = a + b
 * @param req 请求参数
 * @param res 返回参数
 */
void add(rclcpp::Node::SharedPtr node,
         beginner_tutorials::srv::AddTwoInts::Request::SharedPtr req,
         beginner_tutorials::srv::AddTwoInts::Response::SharedPtr res) {
  res->sum = req->a + req->b;
  RCLCPP_INFO(node->get_logger(), "add_two_ints: %ld + %ld, response %ld",
              req->a, req->b, res->sum);
}

int main(int argc, char **argv) {
  // 初始化 ros 节点
  rclcpp::init(argc, argv);
  // 创建一个 ros 节点句柄
  rclcpp::Node::SharedPtr node =
      rclcpp::Node::make_shared("add_two_ints_server");
  /**
   * 创建一个服务提供者对象
   *
   * 通过 NodeHandle::advertiseService 向 ros 的 master 节点 注册一个 名称为
   * add_two_ints 的 服务, add 函数作为服务的处理函数. advertiseService 返回一个
   * 服务提供者实例, 在所有服务提供者实例及其拷贝 被销毁后, 相应的服务将会从 ros
   * master 中注销.
   */
  rclcpp::Service<beginner_tutorials::srv::AddTwoInts>::SharedPtr server =
      node->create_service<beginner_tutorials::srv::AddTwoInts>(
          "add_two_ints", std::bind(add, node, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(node->get_logger(), "ready to serve add_two_ints service");
  // 进入 ros 的消息循环
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}