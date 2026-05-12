#include "global_planner_normal.h"

namespace Planning
{
    GlobalPlannerNormal::GlobalPlannerNormal() // 普通全局路径规划器
    {
        RCLCPP_INFO(rclcpp::get_logger("global_path"), "global_planner_normal created");
        global_planner_config_ = std::make_unique<ConfigReader>();
        global_planner_config_->read_global_path_config();
        global_planner_type_=static_cast<int>(GlobalPlannerType::NORMAL);

    }
       Path GlobalPlannerNormal::search_global_path(const PNCMap &pnc_map)
    {

        RCLCPP_INFO(rclcpp::get_logger("global_path"), "search_global_path called");
        init_global_path(pnc_map);
        return global_path_;
    }

     
    void GlobalPlannerNormal::init_global_path(const PNCMap &pnc_map)
    {
        global_path_.header.frame_id="map";
        global_path_.header.stamp=rclcpp::Clock().now();
        global_path_.poses.clear();

        PoseStamped p_tmp;
        p_tmp.header=global_path_.header;
        p_tmp.pose.orientation.x=0.0;
        p_tmp.pose.orientation.y=0.0;
        p_tmp.pose.orientation.z=0.0;  
        p_tmp.pose.orientation.w=1.0;
        
        const int midline_size = pnc_map.midline.points.size();
        for (size_t i = 0; i < midline_size; ++i)
        {
            p_tmp.pose.position.x = (pnc_map.midline.points[i].x+pnc_map.right_boundary.points[i].x)/2.0;
            p_tmp.pose.position.y = (pnc_map.midline.points[i].y+pnc_map.right_boundary.points[i].y)/2.0;
            p_tmp.pose.position.z = 0.0;
            global_path_.poses.emplace_back(p_tmp);
        }
        RCLCPP_INFO(rclcpp::get_logger("global_path"), "Global path initialized with %zu poses", global_path_.poses.size());


    }

} // namespace Planning