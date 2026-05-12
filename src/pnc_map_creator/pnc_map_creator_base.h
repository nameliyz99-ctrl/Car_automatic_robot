#ifndef PNC_MAP_CREATOR_BASE_H_
#define PNC_MAP_CREATOR_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <cmath>
#include "config_reader.h"

namespace Planning
{   using base_msgs::msg::PNCMap;
    using geometry_msgs::msg::Point;
    using visualization_msgs::msg::MarkerArray;
    using visualization_msgs::msg::Marker;
    
    enum class PNCMapType
    {
        STRAIGHT, // 直道
        STURN,   // S弯    
        // 后续可以添加更多地图类型，如环岛、十字路口等
    };    
    class PNCMapCreatorBase // pnc_map创建器基类
    {
    public:
            virtual PNCMap creat_pnc_map() = 0; // 创建pnc_map的纯虚函数，具体实现由子类完成
            inline PNCMap pnc_map() const { return pnc_map_; } // 获取创建的pnc_map对象
            inline MarkerArray pnc_map_markerarray() const { return pnc_map_markerarray_; } // 获取创建的pnc_map对应的MarkerArray对象
            virtual ~PNCMapCreatorBase() = default; // 虚析构函数，确保子类对象被正确销毁
    protected:
         std::unique_ptr<ConfigReader> pnc_map_config_; // 配置读取器，子类可以使用它来读取地图创建相关的配置参数
         int map_type_=0; // 地图类型
         PNCMap pnc_map_; // 创建的pnc_map对象，子类在创建地图时可以直接操作这个对象
         MarkerArray pnc_map_markerarray_; // 创建的pnc_map对应的MarkerArray，子类在创建地图时可以直接操作这个对象来生成可视化Marker

        Point p_mid_,pl_,pr_; // 地图中点坐标，子类在创建地图时可以使用这个点作为参考来生成地图元素的位置
        double theta_current_=0.0; // 地图当前朝向，子类在创建地图时可以使用这个角度来确定地图元素的朝向
        double len_step_=0.0; // 直道长度，子类在创建直道地图时可以使用这个参数来确定直道的长度
        double theta_step_=0.0; // S弯长度，子类在创建S弯地图时可以使用这个参数来确定S弯的长度
    };
} // namespace Planning
#endif // PNC_MAP_CREATOR_BASE_H_
