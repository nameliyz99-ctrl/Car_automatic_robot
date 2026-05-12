#ifndef CAR_MOVE_CMD_H_
#define CAR_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_trajectory.hpp"
#include "geometry_msgs/msg/transform_stamped.h"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>
#include "config_reader.h"
#include "main_car_info.h"

namespace Planning
{

    using namespace std::chrono_literals;
    using base_msgs::msg::LocalTrajectory;
    using geometry_msgs::msg::TransformStamped;
    using std::placeholders::_1;
    using tf2_ros::TransformBroadcaster;

    struct car_param
    {
        double pos_x_ = 0.0;
        double pos_y_ = 0.0;
        double theta_ = 0.0;
        double speed_ = 0.0;
    };

    class CarMoveCmd : public rclcpp::Node // 主车运动指令
    {
    public:
        CarMoveCmd();
    private:
        void car_broadcaster_tf(const LocalTrajectory::SharedPtr trajectory); // TF广播回调函数

    private:
        std::unique_ptr<ConfigReader> move_cmd_config_;                      // 配置读取器
        std::shared_ptr<TransformBroadcaster> broadcaster_;                     // TF监听器
        rclcpp::Subscription<LocalTrajectory>::SharedPtr local_trajectory_sub_; // 局部轨迹订阅器
        std::shared_ptr<VehicleBase> car_;                                   // 主车
        car_param car_param_;                                                // 主车状态
    };

} // namespace Planning
#endif // CAR_MOVE_CMD_H_
