/*
文件名: 规划总流程入口
作者: C哥智驾说
完成时间: 2024.12

编译类型: 节点(planning_process)

流程: 为节点planning_process提供入口函数main()

Copyright © 2024 C哥智驾说 All rights reserved.
版权所有 侵权必究
*/
// #ifndef PLANNING_PROCESS_MAIN_H
// #define PLANNING_PROCESS_MAIN_H
#include "planning_process.h"

// namespace Planning
// {
//   class PlanningProcess : public rclcpp::Node
//   {
//   public:
//     PlanningProcess() : Node("planning_process")
//     {
//       RCLCPP_INFO(this->get_logger(), "PlanningProcess node has been created.");
//     }

//     bool process();{return true;} // 规划总流程
//     private:
//   };
// } // namespace Planning
// #endif  

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("planning_process_main"), "planning start");

  auto node = std::make_shared<Planning::PlanningProcess>();
  if(!node->process())
  {
    RCLCPP_ERROR(rclcpp::get_logger("planning_process_main"), "planning failed!");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
