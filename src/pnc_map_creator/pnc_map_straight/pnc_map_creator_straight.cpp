#include "pnc_map_creator_straight.h"

namespace Planning
{
    PNCMapCreatorStraight::PNCMapCreatorStraight() // 直道地图
    {
        RCLCPP_INFO(rclcpp::get_logger("pnc_map"), "pnc_map_straight_create created");
        // 读取配置文件，给参数赋值
        pnc_map_config_ = std::make_unique<ConfigReader>();
        pnc_map_config_->read_pnc_map_config();
        map_type_ = static_cast<int>(PNCMapType::STRAIGHT);

        // 地图起点坐标
        p_mid_.x = -3.0;
        p_mid_.y = pnc_map_config_->pnc_map().road_half_width_ / 2.0;
        // 长度步长
        len_step_ = pnc_map_config_->pnc_map().segment_len_;
        // 地图初始化
        init_pnc_map();
    }
    PNCMap PNCMapCreatorStraight::creat_pnc_map() // 生成地图
    {
        draw_straight_x(pnc_map_.road_length, 1.0);
        // 保证pnc_map_.midline.points数量为偶数
        if (pnc_map_.midline.points.size() % 2 == 1)
        {
            pnc_map_.midline.points.pop_back();
        }

        // 把maker放到makerarray中
        pnc_map_markerarray_.markers.emplace_back(pnc_map_.midline);
        pnc_map_markerarray_.markers.emplace_back(pnc_map_.left_boundary);
        pnc_map_markerarray_.markers.emplace_back(pnc_map_.right_boundary);
        RCLCPP_INFO(rclcpp::get_logger("pnc_map"), "pnc_map_straight_create, midline points %ld", pnc_map_.midline.points.size());

        return pnc_map_;
    }

    void PNCMapCreatorStraight::init_pnc_map()
    {
        pnc_map_.header.frame_id = pnc_map_config_->pnc_map().frame_;
        pnc_map_.header.stamp = rclcpp::Clock().now();
        pnc_map_.road_length = pnc_map_config_->pnc_map().road_length_;
        pnc_map_.road_half_width = pnc_map_config_->pnc_map().road_half_width_;

        // 中心线格式
        pnc_map_.midline.header = pnc_map_.header;
        pnc_map_.midline.ns = "pnc_map";
        pnc_map_.midline.id = 0;
        pnc_map_.midline.action = Marker::ADD;
        pnc_map_.midline.type = Marker::LINE_LIST; // 分段线条
        pnc_map_.midline.scale.x = 0.05;           // 线宽
        pnc_map_.midline.color.a = 1.0;            // 不透明度
        pnc_map_.midline.color.r = 0.7;
        pnc_map_.midline.color.g = 0.7;
        pnc_map_.midline.color.b = 0.0;
        pnc_map_.midline.lifetime = rclcpp::Duration::max();
        pnc_map_.midline.frame_locked = true;
        // 左边线格式
        pnc_map_.left_boundary = pnc_map_.midline;
        pnc_map_.left_boundary.id = 1;
        pnc_map_.left_boundary.type = Marker::LINE_STRIP;
        pnc_map_.left_boundary.color.r = 1.0;
        pnc_map_.left_boundary.color.g = 1.0;
        pnc_map_.left_boundary.color.b = 1.0;

        // 右边线格式
        pnc_map_.right_boundary = pnc_map_.left_boundary;
        pnc_map_.right_boundary.id = 2;
    }

    void PNCMapCreatorStraight::draw_straight_x(const double &lenght, const double &plus_flag, const double &ratio)
    {
        double len_tmp = 0.0;
        while (len_tmp < lenght)
        {
            pl_.x = p_mid_.x;
            pl_.y = p_mid_.y + pnc_map_config_->pnc_map().road_half_width_;
            pr_.x = p_mid_.x;
            pr_.y = p_mid_.y - pnc_map_config_->pnc_map().road_half_width_;

            pnc_map_.midline.points.emplace_back(p_mid_);
            pnc_map_.left_boundary.points.emplace_back(pl_);
            pnc_map_.right_boundary.points.emplace_back(pr_);

            len_tmp += len_step_ * ratio;
            // p_mid_.x += len_tmp * plus_flag;
            p_mid_.x += len_step_ * plus_flag * ratio;
        }
    }

} // namespace Planning