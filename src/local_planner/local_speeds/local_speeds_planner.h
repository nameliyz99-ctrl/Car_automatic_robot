#ifndef LOCAL_SPEED_PLANNER_H_
#define LOCAL_SPEED_PLANNER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_speeds_point.hpp"
#include "base_msgs/msg/local_speeds.hpp"

#include "config_reader.h"
#include "polynomial_curve.h"
#include "decision_center.h"
#include "local_speeds_smoother.h"

namespace Planning
{
    using base_msgs::msg::LocalSpeeds;
    using base_msgs::msg::LocalSpeedsPoint;

    class LocalSpeedsPlanner // 速度规划器
    {
    public:
        LocalSpeedsPlanner();
        LocalSpeeds cal_speed(const std::shared_ptr<DecisionCenter> &decision); // 计算速度规划
    private:
        void init_local_speeds();                                         // 初始化速度规划
        inline LocalSpeeds local_speeds() const { return local_speeds_; } // 获取速度规划
    private:
        std::unique_ptr<ConfigReader> local_speeds_config_;
        std::shared_ptr<LocalSpeedsSmoother> local_speeds_smoother_;
        LocalSpeeds local_speeds_;
        
        const double min_speed = 0.1; // 时间步长，单位为秒，根据实际需求调整
    };
} // namespace Planning
#endif // LOCAL_SPEED_PLANNER_H_