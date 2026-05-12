#include "obs_car_info.h"

namespace Planning
{
    ObsCar::ObsCar(const int &id) // 障碍物车辆
    {
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "obs_car created");

        vehicle_config_ = std::make_unique<ConfigReader>();
        vehicle_config_->read_vehicles_config();

        child_frame_ = vehicle_config_->obs_pair()[id].frame_;
        length_ = vehicle_config_->obs_pair()[id].length_;
        width_ = vehicle_config_->obs_pair()[id].width_;
        theta_ = vehicle_config_->obs_pair()[id].pose_theta_;
        speed_ = vehicle_config_->obs_pair()[id].speed_ori_;
        id_ = id;

        tf2::Quaternion qtn;
        qtn.setRPY(0, 0, theta_);
        loc_point_.header.frame_id = vehicle_config_->pnc_map().frame_;
        loc_point_.header.stamp = rclcpp::Clock().now();
        loc_point_.pose.position.x = vehicle_config_->obs_pair()[id].pose_x_;
        loc_point_.pose.position.y = vehicle_config_->obs_pair()[id].pose_y_;
        loc_point_.pose.position.z = 0.0;
        loc_point_.pose.orientation.x = qtn.getX();
        loc_point_.pose.orientation.y = qtn.getY();
        loc_point_.pose.orientation.z = qtn.getZ();
        loc_point_.pose.orientation.w = qtn.getW();
    }
    void ObsCar::vechicle_cartesin_to_frent(const Referline &refer_line)
    {
        ToFrenetInPutTP point_in_referline;
        // 计算定位点在参考线上的投影点
        Curve::find_projection_point(refer_line, loc_point_, point_in_referline);
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "obs_car project_point: rs = %f, rx = %f, ry = %f, rtheta = %f, rkappa = %f, rdkappa = %f",
                    point_in_referline.rs, point_in_referline.rx, point_in_referline.ry,
                    point_in_referline.rtheta, point_in_referline.rkappa, point_in_referline.rdkappa);
        // 计算定位点在frent坐标系下的参数
        point_in_referline.x = loc_point_.pose.position.x;
        point_in_referline.y = loc_point_.pose.position.y;
        point_in_referline.speed = speed_;
        point_in_referline.theta = theta_;
        point_in_referline.kappa = kappa_;
        Curve::cartensian_to_frenet(point_in_referline, to_path_frenet_params_);
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "obs_car cartesian_to_frent: s = %f, ds_dt = %f, dds_dt = %f, l = %f, dl_ds = %f, dl_ds = %f, ddl_ds = %f, dl_dt = %f, ddl_dt = %f",
                    to_path_frenet_params_.s, to_path_frenet_params_.ds_dt,
                    to_path_frenet_params_.dds_dt, to_path_frenet_params_.l,
                    to_path_frenet_params_.dl_ds, to_path_frenet_params_.dl_dt,
                    to_path_frenet_params_.ddl_ds, to_path_frenet_params_.ddl_dt, to_path_frenet_params_.ddl_dt);
    }
    void ObsCar::vechicle_cartesin_to_frent_2path(const LocalPath &local_path, const Referline &refer_line,
                                                  const std::shared_ptr<VehicleBase> &car)
    {

        const double path0_index = Curve::find_match_point(refer_line, local_path.local_path[0].pose);
        const double path_end_index = Curve::find_match_point(refer_line, local_path.local_path.back().pose);

        // 计算历经起点终点在参考线下的下标
        // 当障碍物在参考喜爱年上的s值超出路经首位范围时
        if (s_ > refer_line.refer_line[path_end_index].rs || s_ < refer_line.refer_line[path0_index].rs)
        { // 超出路径前端
            s_2path_ = s_ - refer_line.refer_line[path0_index].rs;
            ds_dt_2path_ = ds_dt_;
            l_2path_ = l_ - car->l();
            dl_ds_2path_ = dl_ds_;
            dl_dt_2path_ = dl_dt_;
            dds_dt_2path_ = dds_dt_;
            ddl_ds_2path_ = ddl_ds_;
            ddl_dt_2path_ = ddl_dt_;
            RCLCPP_INFO(rclcpp::get_logger("vehicle"), "obs_car 2path approx: s_2path = %f, ds_dt_2path = %f, dds_dt_2path = %f, l_2path = %f, dl_ds_2path = %f, dl_dt_2path = %f, ddl_ds_2path = %f, ddl_dt_2path = %f",
                        s_2path_, ds_dt_2path_, dds_dt_2path_, l_2path_, dl_ds_2path_, dl_dt_2path_, ddl_ds_2path_, ddl_dt_2path_);
            return;
        }
        ToFrenetInPutTP projection_point; // 计算定位点在路径上的投影点
        Curve::find_projection_point(local_path, loc_point_, projection_point);
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "obs_car projection_point to path: rs = %.2f, rx = %.2f, ry = %.2f, rtheta = %.2f, rkappa = %.2f, rdkappa = %.2f",
                    projection_point.rs, projection_point.rx, projection_point.ry, projection_point.rtheta, projection_point.rkappa, projection_point.rdkappa);

        // 计算定位点在frenet坐标下的参数
        ToFrenetInPutTP frenet_input;
        frenet_input.x = loc_point_.pose.position.x;
        frenet_input.y = loc_point_.pose.position.y;
        frenet_input.theta = theta_;
        frenet_input.speed = speed_;
        frenet_input.a = acceleration_;
        frenet_input.kappa = dkappa_;
        frenet_input.rs = projection_point.rs;
        frenet_input.rx = projection_point.rx;
        frenet_input.ry = projection_point.ry;
        frenet_input.rtheta = projection_point.rtheta;
        frenet_input.rkappa = projection_point.rkappa;
        frenet_input.rdkappa = projection_point.rdkappa;
        ToFrenetOutTP frenet_output;
        Curve::cartensian_to_frenet(frenet_input, frenet_output);
        s_2path_ = frenet_output.s;
        ds_dt_2path_ = frenet_output.ds_dt;
        dds_dt_2path_ = frenet_output.dds_dt;
        l_2path_ = frenet_output.l;
        dl_ds_2path_ = frenet_output.dl_ds;
        dl_dt_2path_ = frenet_output.dl_dt;
        ddl_ds_2path_ = frenet_output.ddl_ds;
        ddl_dt_2path_ = frenet_output.ddl_dt;
        RCLCPP_INFO(rclcpp::get_logger("vehicle"), "obs_car cartesian_to_frenet to path: s_2path = %.2f, ds_dt_2path = %.2f, l_2path = %.2f",
                    s_2path_, ds_dt_2path_, l_2path_);
    }

} // namespace Planning