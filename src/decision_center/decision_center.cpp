#include "decision_center.h"

namespace Planning
{
 DecisionCenter::DecisionCenter()
    {
        RCLCPP_INFO(rclcpp::get_logger("decision_center"), "decision_center created");

        // 读取配置文件
        decision_config_ = std::make_unique<ConfigReader>();
        decision_config_->read_decision_config();
    }

    void DecisionCenter::make_path_decision(const std::shared_ptr<VehicleBase> &car, const std::vector<std::shared_ptr<VehicleBase>> &obses)
    {
        if (obses.empty())
        {
            return;
        }
        // 初始化
        sl_points_.clear();
        const double left_bound_l = decision_config_->pnc_map().road_half_width_ * 1.5;              // 道路左边界
        const double right_bound_l = -decision_config_->pnc_map().road_half_width_ * 0.5;            // 道路右边界
        const double dis_time = static_cast<double>(decision_config_->local_path().path_size_ - 50); // 开始考虑障碍物的范围  提前size-50个点
        const double least_length = std::max(car->to_path_frenet_params().ds_dt * dis_time, 30.0);   // 最小变道距离 根据车速调整距离，增加最近约束
        const double referline_end_length = decision_config_->refer_line().front_size_ *
                                            decision_config_->pnc_map().segment_len_; // 参考线前端的长度最大值
        SLPoint p;
        // 对每个障碍物计算变道点位
        for (const auto &obs : obses)
        {
            const double obs_dis_s = obs->to_path_frenet_params().s - car->to_path_frenet_params().s; // 与障碍物距离
            if (obs_dis_s > referline_end_length ||                                                   // 障碍物在参考线末端的距离 （接近目标终点，参考线变短，也要通过最长距离判断）
                obs_dis_s < -least_length)                                                            // 后方比较远的障碍物
            {
                continue;
            }
            if (obs->to_path_frenet_params().l > right_bound_l && obs->to_path_frenet_params().l < left_bound_l &&                                     // 障碍物在车道横向中间
                fabs(obs->to_path_frenet_params().dl_dt) < min_speed && obs->to_path_frenet_params().ds_dt < car->to_path_frenet_params().ds_dt / 2.0) // 障碍物横向速度为0，纵向速度慢
            {
                p.s_ = obs->to_path_frenet_params().s + obs->to_path_frenet_params().ds_dt * obs_dis_s / (car->to_path_frenet_params().ds_dt - obs->to_path_frenet_params().ds_dt);
                const double obs_left_bound_l = obs->to_path_frenet_params().l + obs->width() / 2.0;  // 障碍物左边界
                const double obs_right_bound_l = obs->to_path_frenet_params().l - obs->width() / 2.0; // 障碍物右边界
                const double left_width = left_bound_l - obs_left_bound_l;                            // 左边宽度
                const double right_width = obs_right_bound_l - right_bound_l;                         // 右边宽度

                if (left_width > car->width() + decision_config_->decision().safe_dis_l_ * 2.0) // 左边可以通过
                {
                    p.l_ = (left_bound_l + obs_left_bound_l) / 2.0;
                    p.type_ = static_cast<int>(SLPointType::LEFT_PASS);
                    sl_points_.push_back(p);
                }
                else
                {

                    if (right_width > car->width() + decision_config_->decision().safe_dis_l_ * 2.0) // 右边可以通过
                    {
                        p.l_ = (right_bound_l + obs_right_bound_l) / 2.0;
                        p.type_ = static_cast<int>(SLPointType::RIGHT_PASS);
                        sl_points_.push_back(p);
                    }
                    else // 两边宽度都不够
                    {
                        p.l_ = 0.0;
                        p.s_ = obs->to_path_frenet_params().s - decision_config_->decision().safe_dis_s_;
                        p.type_ = static_cast<int>(SLPointType::STOP);
                        sl_points_.push_back(p);
                        RCLCPP_INFO(rclcpp::get_logger("decision_center"), "stop obs p:(s = %.2f, l = %.2f)", p.s_, p.l_);
                        break; // 更前面的不考虑
                    }
                }
            }
        }
        if (sl_points_.empty())
        {
            return;
        }
        // 头尾的处理
        SLPoint p_start;
        p_start.s_ = sl_points_[0].s_ - least_length;
        p_start.l_ = 0.0;
        p_start.type_ = static_cast<int>(SLPointType::START);
        sl_points_.emplace(sl_points_.begin(), p_start); // 头插
        if (sl_points_.back().type_ != static_cast<int>(SLPointType::END))
        {
            SLPoint p_end;
            p_end.s_ = sl_points_.back().s_ + least_length;
            p_end.l_ = 0.0;
            p_end.type_ = static_cast<int>(SLPointType::END);
            sl_points_.emplace_back(p_end);
        }
    }

    void DecisionCenter::make_speed_decision(const std::shared_ptr<VehicleBase> &car,
                                             const std::vector<std::shared_ptr<VehicleBase>> &obses)
    {
        if (obses.empty())//没有障碍物，不需要变速决策
        {
            return;
        }
        // 初始化
        st_points_.clear();
        const double ori_dis_time = static_cast<double>(decision_config_->local_speeds().speed_size_ - 50); // 开始考虑障碍物的范围  提前size-50个点
        const double ori_dis = ori_dis_time * decision_config_->main_car().speed_ori_;//开始考虑障碍物的范围距离50m
        const double real_brake_time = (ori_dis_time + static_cast<double>(decision_config_->local_speeds().speed_size_)) / 2.0;// 实际刹车时间，取决于配置的刹车点数量和开始考虑障碍物的时间，增加约束
        STPoint p;
        // 对每个障碍物计算变速点位
        for (const auto &obs : obses)
        {
            const double obs_dis_s = obs->s_2path() + car->speed();    // 与障碍物距离
            if (obs_dis_s > ori_dis ||                                 // 障碍物在参考线末端的距离
                obs_dis_s < -decision_config_->decision().safe_dis_s_) // 后方比较远的障碍物
            {
                continue;
            }

            double t_in;
            double t_out;

            if (fabs(obs->l_2path()) < obs->width() / 2.0) // 障碍物在已经占据路径
            {

                if (fabs(obs->ds_dt_2path()) < min_speed) // 障碍物速度慢，需要减速
                {
                    if (fabs(obs->ds_dt_2path() > car->speed() + 0.5))
                    {
                        continue; // 障碍物快，但比车速快很多，不考虑
                    }
                    obs->update_t0();
                    p.t0_ = obs->t0();
                    p.s0_ = obs_dis_s + obs->ds_dt_2path() * p.t0_ - ori_dis;
                    t_in = 0.0;
                    t_out = decision_config_->local_speeds().speed_size_;
                    RCLCPP_INFO(rclcpp::get_logger("decision_center"), "obs_dis_s = %.2f, obs_speed = %.2f, brake_time = %.2f, t_in = %.2f, t_out = %.2f", obs_dis_s, obs->ds_dt_2path(), real_brake_time, t_in, t_out);
                    // 计算s和t点
                    p.t_ = p.t0_ + real_brake_time;
                    p.s_2path_ = obs_dis_s - decision_config_->decision().safe_dis_s_ + obs->ds_dt_2path() * p.t_;
                    p.ds_dt_2path_ = obs->ds_dt_2path();//跟车
                    p.type_ = static_cast<int>(STPointType::STOP);
                    st_points_.emplace_back(p);
                    RCLCPP_INFO(rclcpp::get_logger("decision_center"), "stop obs p:(t = %.2f, s_2path = %.2f, ds_dt_2path = %.2f, t0 = %.2f, s0 = %.2f)", p.t_, p.s_2path_, p.ds_dt_2path_, p.t0_, p.s0_);
                    obs->update_t_in_out(p.t_, t_in, t_out);
                    break;
                }
            }
            else // 横穿
            {

                if (fabs(obs->ds_dt_2path() < min_speed))
                {
                    continue; // 障碍物bu横穿，但速度很慢，不考虑
                }

                if (decision_config_->main_car().speed_ori_ < min_speed)
                {
                    continue;
                }

                const double car_dis_time = obs_dis_s / decision_config_->main_car().speed_ori_;
                const double obs_dis_time = (0.0 - obs->l_2path()) / obs->ds_dt_2path();
                if (car_dis_time < 0.0)
                {
                    continue; // 预计碰撞时间较短，来不及反应，不考虑
                }
                obs->update_t0();
                p.t0_ = obs->t0();
                p.s0_ = obs_dis_s - ori_dis;

                const double delta_t = decision_config_->decision().safe_dis_s_ / decision_config_->main_car().speed_ori_;
                const double half_through_time = fabs(obs->length() / 2.0/ obs->ds_dt_2path());
                t_in = obs_dis_time - half_through_time;
                t_out = obs_dis_time + half_through_time;
                RCLCPP_INFO(rclcpp::get_logger("decision_center"), "obs_dis_s = %.2f, obs_speed = %.2f, cross_time = %.2f, t_in = %.2f, t_out = %.2f", obs_dis_s, obs->ds_dt_2path(), obs_dis_time, t_in, t_out);
                if (car_dis_time > obs_dis_time && car_dis_time < t_out + delta_t) // rangxing
                {
                    p.t_ = t_out;
                    p.s_2path_ = obs_dis_s - decision_config_->decision().safe_dis_s_;
                    p.ds_dt_2path_ = decision_config_->main_car().speed_ori_;
                    p.type_ = static_cast<int>(STPointType::GIVE_WAY);
                    st_points_.emplace_back(p);
                    RCLCPP_INFO(rclcpp::get_logger("decision_center"), "give way obs p:(t = %.2f, s_2path = %.2f)", p.t_, p.s_2path_);
                }
                else if (car_dis_time < obs_dis_time && car_dis_time > t_in - delta_t)//强行
                {
                    p.t_ = t_in;
                    p.s_2path_ = obs_dis_s + decision_config_->decision().safe_dis_s_;
                    p.ds_dt_2path_ = decision_config_->main_car().speed_ori_;
                    p.type_ = static_cast<int>(STPointType::RUSH_OUT);
                    st_points_.emplace_back(p);
                    RCLCPP_INFO(rclcpp::get_logger("decision_center"), "rush out obs p:(t = %.2f, s_2path = %.2f)", p.t_, p.s_2path_);
                }
                obs->update_t_in_out(p.t_, t_in, t_out);
            }
        }
        if (st_points_.empty())
        {
            return;
        }

        // 整个过程的起点和终点 
        STPoint p_start;
        p_start.t_ = st_points_[0].t0_;
        p_start.s_2path_ = st_points_[0].s0_;
        p_start.ds_dt_2path_ = decision_config_->main_car().speed_ori_;
        p_start.type_ = static_cast<int>(STPointType::START);
        st_points_.emplace(st_points_.begin(), p_start); // 头插
        STPoint p_end;
        p_end.t_ = decision_config_->local_speeds().speed_size_;
        p_end.s_2path_ = st_points_.back().s_2path_ + st_points_.back().ds_dt_2path_ * (p_end.t_ - st_points_.back().t_);
        p_end.ds_dt_2path_ = st_points_.back().ds_dt_2path_;
        p_end.type_ = static_cast<int>(STPointType::END);
        st_points_.emplace_back(p_end);
        RCLCPP_INFO(rclcpp::get_logger("decision_center"), "end obs p:(t = %.2f, s_2path = %.2f)", p_end.t_, p_end.s_2path_);
    }

} // namespace Planning