#include "local_speeds_smoother.h"

namespace Planning
{
  LocalSpeedsSmoother::LocalSpeedsSmoother() // 速度平滑器
  {
    local_speeds_config_ = std::make_unique<ConfigReader>();
    local_speeds_config_->read_local_speeds_config();
    RCLCPP_INFO(rclcpp::get_logger("local_speed"), "local_speeds_smoother created");
  }

  void LocalSpeedsSmoother::smooth_local_speeds(LocalSpeeds &speeds)
  {
    (void)speeds;
    // 速度平滑算法实现
    // 这里可以使用一些常见的平滑算法，如移动平均、卡尔曼滤波等，根据实际需求选择合适的算法进行实现
    RCLCPP_INFO(rclcpp::get_logger("local_speed"), "smoothing local speeds...");
  }

} // namespace Planning