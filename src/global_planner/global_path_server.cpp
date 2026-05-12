#include "global_path_server.h"

namespace Planning
{
    GlobalPathServer::GlobalPathServer() : Node("global_path_server_node") // 全局路径服务器
    {
        RCLCPP_INFO(this->get_logger(), "global_path_server_node created");

        global_path_pub_ = this->create_publisher<Path>("global_path", 10);             // 创建全局路径发布器
        global_path_rviz_pub_ = this->create_publisher<Marker>("global_path_rviz", 10); // 创建全局路径RViz可视化发布器
        global_path_server_ = this->create_service<GlobalPathService>(
            "global_path_service",
            std::bind(&GlobalPathServer::response_global_path_callback, this, std::placeholders::_1, std::placeholders::_2)); // 创建全局路径服务
    }

    void GlobalPathServer::response_global_path_callback(const std::shared_ptr<GlobalPathService::Request> request,
                                                         const std::shared_ptr<GlobalPathService::Response> response)
    {
        switch (request->global_planner_type)
        {
        case static_cast<int>(GlobalPlannerType::NORMAL):
            global_planner_base_ = std::make_shared<GlobalPlannerNormal>(); // 创建普通全局路径规划器
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Unknown global planner type requested: %d", request->global_planner_type);
            return;
        }  
            if (request->pnc_map.midline.points.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Received empty PNC map, cannot plan path");
                return;
            }

            const auto global_path = global_planner_base_->search_global_path(request->pnc_map); // 规划全局路径
            response->global_path = global_path;                                                 // 填充响应

            global_path_pub_->publish(global_path); // 发布全局路径
            RCLCPP_INFO(this->get_logger(), "Published global path with %zu poses", global_path.poses.size());

            const auto global_path_rviz = path2marker(global_path); // 将全局路径转换为Marker用于RViz可视化
            global_path_rviz_pub_->publish(global_path_rviz);       // 发布全局路径RViz可视化
            RCLCPP_INFO(this->get_logger(), "Published global path visualization with %zu markers", global_path_rviz.points.size());
        
    }

    Marker GlobalPathServer::path2marker(const Path &path)
    {
        
            Marker path_rviz_;
            path_rviz_.header = path.header;
            path_rviz_.ns = "global_path";
            path_rviz_.id = 0;
            path_rviz_.type = Marker::LINE_STRIP;
            path_rviz_.action = Marker::ADD;
            path_rviz_.scale.x = 0.05;
            path_rviz_.color.a = 1.0;
            path_rviz_.color.r = 0.8;
            path_rviz_.color.g = 0.0;
            path_rviz_.color.b = 0.0;
            path_rviz_.lifetime = rclcpp::Duration::max();
            path_rviz_.frame_locked = true;

            Point p_tmp;
            for (const auto &pose : path.poses)
            {
                p_tmp.x = pose.pose.position.x;
                p_tmp.y = pose.pose.position.y;
                // p_tmp.z = pose.pose.position.z;
                path_rviz_.points.emplace_back(p_tmp);
            }
            return path_rviz_;
    }

} // namespace Planning

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::GlobalPathServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}