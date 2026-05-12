#include "local_path_smoother.h"
namespace Planning
{
    LocalPathSmoother::LocalPathSmoother()
    { // 局部路径平滑器
        RCLCPP_INFO(rclcpp::get_logger("local_path"), "local_path_smoother created");
        local_path_config_ = std::unique_ptr<ConfigReader>();
        local_path_config_->read_local_path_config();
    }

    // 路径平滑  todo：johan
    void LocalPathSmoother::smooth_local_path(LocalPath &path)
    {

        RCLCPP_INFO(rclcpp::get_logger("local_path"), "local path smoothed");
        (void)path;
    }
} // namespace Planning
