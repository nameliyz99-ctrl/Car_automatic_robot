#include "main_car_info.h"

namespace Planning
{
    MainCar::MainCar() // 主车信息
    {
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "main_car created");
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "main_car created");

        // 读取配置
        vehicle_config_ = std::make_unique<ConfigReader>();
        vehicle_config_->read_vehicles_config();

        // 更新基本属性
        child_frame_ = vehicle_config_->main_car().frame_;
        length_ = vehicle_config_->main_car().length_;
        width_ = vehicle_config_->main_car().width_;
        id_ = vehicle_config_->main_car().id_;
        theta_ = vehicle_config_->main_car().pose_theta_;
        speed_ = vehicle_config_->main_car().speed_ori_;

        // 初始化定位点
        tf2::Quaternion qtn;
        qtn.setRPY(0, 0, theta_);
        loc_point_.header.frame_id = vehicle_config_->pnc_map().frame_;
        loc_point_.header.stamp = rclcpp::Clock().now();
        loc_point_.pose.position.x = vehicle_config_->main_car().pose_x_;
        loc_point_.pose.position.y = vehicle_config_->main_car().pose_y_;
        loc_point_.pose.position.z = 0.0;
        loc_point_.pose.orientation.x = qtn.getX();
        loc_point_.pose.orientation.y = qtn.getY();
        loc_point_.pose.orientation.z = qtn.getZ();
        loc_point_.pose.orientation.w = qtn.getW();
    }
    void MainCar::vechicle_cartesin_to_frent(const Referline &refer_line)
    {
        ToFrenetInPutTP point_in_referline;
        // 计算定位点在参考线上的投影点
        Curve::find_projection_point(refer_line, loc_point_, point_in_referline);
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "main_car project_point: rs = %f, rx = %f, ry = %f, rtheta = %f, rkappa = %f, rdkappa = %f",
                    point_in_referline.rs, point_in_referline.rx, point_in_referline.ry,
                    point_in_referline.rtheta, point_in_referline.rkappa, point_in_referline.rdkappa);
        // 计算定位点在frent坐标系下的参数
        Curve::cartensian_to_frenet(point_in_referline, to_path_frenet_params_);
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "main_car cartesian_to_frent: s = %f, ds_dt = %f, dds_dt = %f, l = %f, dl_ds = %f, dl_ds = %f, ddl_ds = %f, dl_dt = %f, ddl_dt = %f",
                    to_path_frenet_params_.s, to_path_frenet_params_.ds_dt,
                    to_path_frenet_params_.dds_dt, to_path_frenet_params_.l,
                    to_path_frenet_params_.dl_ds, to_path_frenet_params_.dl_dt,
                    to_path_frenet_params_.ddl_ds, to_path_frenet_params_.ddl_dt, to_path_frenet_params_.ddl_dt);
    }

    void MainCar::vechicle_cartesin_to_frent_2path(const LocalPath &local_path, const Referline &refer_line, const std::shared_ptr<VehicleBase> &car)
    {
        (void)local_path;
        (void)refer_line;
        (void)car;
    }

} // namespace Planning