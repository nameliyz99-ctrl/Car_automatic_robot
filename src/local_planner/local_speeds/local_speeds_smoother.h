#ifndef LOCAL_SPEED_SMOOTHER_H_
#define LOCAL_SPEED_SMOOTHER_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "base_msgs/msg/local_speeds.hpp"
#include "cmath"

namespace Planning
{
    using base_msgs::msg::LocalSpeeds;
    class LocalSpeedsSmoother // 速度平滑器
    {
    public:
        LocalSpeedsSmoother();
        void smooth_local_speeds(LocalSpeeds &speeds); // 速度平滑

    private:
    std::unique_ptr<ConfigReader> local_speeds_config_;
    };
} // namespace Planning
#endif // LOCAL_SPEED_SMOOTHER_H_