#include "obs_move_cmd.h"

namespace Planning
{
  ObsMoveCmd::ObsMoveCmd() : Node("obs_move_cmd_node") // 主车运动指令
  {
    RCLCPP_INFO(this->get_logger(), "obs_move_cmd_node created");

    obs_move_cmd_config_ = std::make_unique<ConfigReader>();
    obs_move_cmd_config_->read_move_cmd_config();

    for (int i = 0; i < 3; i++)
    {
      ObsParam obs_param;
      obs_param.obs_ = std::make_shared<ObsCar>(i + 1);
      obs_param.obs_broadcaster_ = std::make_shared<TransformBroadcaster>(this);

      obs_param.pos_x_ = obs_param.obs_->loc_point().pose.position.x;
      obs_param.pos_y_ = obs_param.obs_->loc_point().pose.position.y;
      obs_param.theta_ = obs_param.obs_->theta();
      obs_param.speed_ = obs_param.obs_->speed();

      obses_params_.emplace_back(obs_param);
      timer_ = this->create_wall_timer(
          100ms,
          std::bind(&ObsMoveCmd::obs_broadcaster_tf, this));
    }
  }

  void ObsMoveCmd::obs_broadcaster_tf()
  {
    for (auto &obs_param : obses_params_)
    {
      TransformStamped transform_data;
      transform_data.header.stamp = this->now();
      transform_data.header.frame_id = obs_move_cmd_config_->pnc_map().frame_;
      transform_data.child_frame_id = obs_param.obs_->child_frame();

      // 赋值
      transform_data.transform.translation.x = obs_param.pos_x_;
      transform_data.transform.translation.x = obs_param.pos_y_;

      const double speed_x = obs_param.speed_ * std::cos(obs_param.theta_);
      const double speed_y = obs_param.speed_ * std::sin(obs_param.theta_);
      obs_param.pos_x_ += speed_x;
      obs_param.pos_y_ += speed_y;

      tf2::Quaternion qtn;
      qtn.setRPY(0, 0, obs_param.theta_);
      transform_data.transform.rotation.x = qtn.x();
      transform_data.transform.rotation.y = qtn.y();
      transform_data.transform.rotation.z = qtn.z();
      transform_data.transform.rotation.w = qtn.w();  

      RCLCPP_INFO(this->get_logger(), "obs %s pos: (%.2f, %.2f), theta: %.2f, speed: %.2f", 
      obs_param.obs_->child_frame().c_str(), obs_param.pos_x_, obs_param.pos_y_, obs_param.theta_, obs_param.speed_);
      obs_param.obs_broadcaster_->sendTransform(transform_data);
      

    }
  }

} // namespace Planning

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Planning::ObsMoveCmd>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}