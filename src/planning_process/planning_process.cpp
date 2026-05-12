#include "planning_process.h"

namespace Planning
{
    PlanningProcess::PlanningProcess() : Node("planning_process") // 规划总流程
    {
        RCLCPP_INFO(this->get_logger(), "planning_process created");

        // 读取配置文件
        process_config_ = std::make_unique<ConfigReader>();
        process_config_->read_planning_process_config();
        obs_dis_ = process_config_->process().obs_dis_;

        car_ = std::make_shared<MainCar>();
        for (int i = 0; i < 3; i++)
        {
            auto obs_car_ = std::make_shared<ObsCar>(i + 1);
            obses_spawn_.emplace_back(obs_car_);
        }
        // 坐标广播器

        tf_broadcaster_ = std::make_shared<StaticTransformBroadcaster>(this);

        // 创建监听器，绑定主车缓存对象
        buffer_ = std::make_unique<Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<TransformListener>(*buffer_, this);

        // 创建地图和全局路径客户端
        map_client_ = this->create_client<PNCMapService>("pnc_map_server");
        global_path_client_ = this->create_client<GlobalPathService>("global_path_service");

        // 创建参考线和参考线发布器
        refer_line_creator_ = std::make_shared<ReferencelineCreator>();
        refer_line_pub_ = this->create_publisher<Path>("reference_line", 10);
        // 创建决策器
        decider_ = std::make_shared<DecisionCenter>();

        // 创建局部路径规划器和发布器
        local_path_planner_ = std::make_shared<LocalPathPlanner>();
        local_speeds_planner_ = std::make_shared<LocalSpeedsPlanner>();
        local_path_pub_ = this->create_publisher<Path>("local_path", 10);

        local_trajectory_combiner_ = std::make_shared<LocalTrajectoryCombiner>();
        local_trajectory_pub_ = this->create_publisher<LocalTrajectory>("local_trajectory", 10);
    }

    bool PlanningProcess::process() // 总流程
    {
        rclcpp::Rate rate(0.2); // 等待一秒
        rate.sleep();

        if (!planning_init())
        {
            RCLCPP_ERROR(this->get_logger(), "planning init failed");
            return false;
        }
        // 进入规划主流程
        timer_ = this->create_wall_timer(
            0.1s,
            std::bind(&PlanningProcess::planning_callback, this));
        return true;
    }
    bool PlanningProcess::planning_init()
    {
        vehicle_spawn(car_); // 车
        for (const auto &obs : obses_spawn_)
        {
            vehicle_spawn(obs);
        }
        // 障碍物车
        //  连接地图服务器
        if (!connect_server(map_client_))
        {
            RCLCPP_ERROR(this->get_logger(), "map server connect failed");

            return false;
        }
        // 获取地图
        if (!map_request())
        {
            RCLCPP_ERROR(this->get_logger(), "map request and response failed");
            return false;
        }
        // 连接全局路径服务器
        if (!connect_server(global_path_client_))
        {
            RCLCPP_ERROR(this->get_logger(), "global path server connect failed");
            return false;
        }
        // 获取全局路径
        if (!global_path_request())
        {
            RCLCPP_ERROR(this->get_logger(), "global path request and response failed");
            return false;
        }

        return true;
    }

    void PlanningProcess::vehicle_spawn(const std::shared_ptr<VehicleBase> &vehicle)
    {

        TransformStamped spawn;
        spawn.header.stamp = this->now();
        spawn.header.frame_id = process_config_->pnc_map().frame_; // 地图坐标
        spawn.child_frame_id = vehicle->child_frame();             // 子坐标设置为车辆坐标

        spawn.transform.translation.x = vehicle->loc_point().pose.position.x;
        spawn.transform.translation.y = vehicle->loc_point().pose.position.y;
        spawn.transform.translation.z = vehicle->loc_point().pose.position.z;
        spawn.transform.rotation.x = vehicle->loc_point().pose.orientation.x;
        spawn.transform.rotation.y = vehicle->loc_point().pose.orientation.y;
        spawn.transform.rotation.z = vehicle->loc_point().pose.orientation.z;
        spawn.transform.rotation.w = vehicle->loc_point().pose.orientation.w;

        RCLCPP_INFO(this->get_logger(), "vehicle %s spawned x = %.2f, y = %.2f",
                    vehicle->child_frame().c_str(),
                    vehicle->loc_point().pose.position.x,
                    vehicle->loc_point().pose.position.y);
        tf_broadcaster_->sendTransform(spawn);
        // 这里不是多肽，因为这些函数不算虚函数或纯虚函数
    }

    void PlanningProcess::get_location(const std::shared_ptr<VehicleBase> &vehicle)
    {

        try
        {
            PoseStamped point;
            auto ts = buffer_->lookupTransform(process_config_->pnc_map().frame_,
                                               vehicle->child_frame(), tf2::TimePointZero);
            point.header = ts.header;
            point.pose.position.x = ts.transform.translation.x;
            point.pose.position.y = ts.transform.translation.y;
            point.pose.position.z = ts.transform.translation.z;
            point.pose.orientation.x = ts.transform.rotation.x;
            point.pose.orientation.y = ts.transform.rotation.y;
            point.pose.orientation.z = ts.transform.rotation.z;
            point.pose.orientation.w = ts.transform.rotation.w;
            vehicle->update_location(point);
        }
        catch (const tf2::LookupException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Lookup exeption: %s", e.what());
        }
    }

    template <typename T>
    bool PlanningProcess::connect_server(const T &client)
    {
        std::string server_name;
        if constexpr (std::is_same_v<T, rclcpp::Client<GlobalPathService>::SharedPtr>)
        {
            server_name = "pnc_map";
        }
        else if constexpr (std::is_same_v<T, rclcpp::Client<PNCMapService>::SharedPtr>)
        {
            server_name = "global_path";
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "unknown server type");
            return false;
        }
        while (!client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the %s service. Exiting.", server_name.c_str());
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for %s service to be available...", server_name.c_str());
        }

        return true;
    }
    bool PlanningProcess::map_request() // 地图请求
    {
        RCLCPP_INFO(this->get_logger(), "Sending map request...");
        auto request = std::make_shared<PNCMapService::Request>();
        request->map_type = process_config_->pnc_map().type_;

        auto result_future = map_client_->async_send_request(request);
        // 判断响应是否成功
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Map request successful");
            pnc_map_ = result_future.get()->pnc_map;
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Map request failed");
            return false;
        }

        return false;
    }
    bool PlanningProcess::global_path_request() // 全局路径请求
    {
        RCLCPP_INFO(this->get_logger(), "Sending global_path request");
        // 生成请求
        auto request = std::make_shared<GlobalPathService::Request>();
        request->pnc_map = pnc_map_;
        request->global_planner_type = process_config_->global_path().type_;

        // 获取响应
        auto result_future = global_path_client_->async_send_request(request);

        // 判断响应是否成功
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "global_path request successful");
            global_path_ = result_future.get()->global_path;
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "global_path request failed");
            return false;
        }
        return false;
        return true;
    }
    void PlanningProcess::planning_callback()
    {
        // 监听车辆定位
        // 参考线
        // 主车和障碍物车辆像参考线投影，计算横向距离和航向差
        // 障碍物按s值排序，计算与主车的距离，判断是否为威胁车辆
        // 根据障碍物距离和威胁等级，选择不同的规划策略
        // 生成新的全局路径，发布参考线和全局路径
        // 速度规划，生成速度曲线，发布速度曲线
        // 合成轨迹，发布轨迹
        const auto start_time = this->get_clock()->now();
        // 监听车辆定位
        get_location(car_);
        obses_.clear();
        // 需要考虑的障碍物
        for (const auto &obs : obses_spawn_)
        {
            get_location(obs);
            if (std::hypot(car_->loc_point().pose.position.x - obs->loc_point().pose.position.x,
                           car_->loc_point().pose.position.y - obs->loc_point().pose.position.y) > obs_dis_)
            {
                continue;
            }
            obses_.emplace_back(obs);
        }

        // 参考线
        // return refer_line_  check ok
        const auto refer_line = refer_line_creator_->create_reference_line(global_path_, car_->loc_point());
        if (refer_line.refer_line.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "reference line is empty");
            return;
        }
        const auto refer_line_rviz = refer_line_creator_->reference_to_rviz();
        refer_line_pub_->publish(refer_line_rviz);

        // 主车和参考线向参考线投影
        car_->vechicle_cartesin_to_frent(refer_line);
        for (const auto &obs : obses_)
        {
            obs->vechicle_cartesin_to_frent(refer_line);
        }

        // 障碍物按s值排序
        std::sort(obses_.begin(), obses_.end(),
                  [](const std::shared_ptr<VehicleBase> &obs1, const std::shared_ptr<VehicleBase> &obs2)
                  { return obs1->to_path_frenet_params().s < obs2->to_path_frenet_params().s; });

        // 路径决策
        decider_->make_path_decision(car_, obses_);

        // 路径规划
        const auto local_path = local_path_planner_->creat_local_path(refer_line, car_, decider_); // 生成局部路径
        if (local_path.local_path.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "local path is empty");
            return;
        }
        const auto local_path_rviz = local_path_planner_->path_to_rviz(); // 生成rviz用的局部路径
        local_path_pub_->publish(local_path_rviz);

        // 障碍物向路径投影
        for (const auto &obs : obses_)
        {
            obs->vechicle_cartesin_to_frent_2path(local_path, refer_line, car_);
        }

        // 速度决策
        decider_->make_speed_decision(car_, obses_);

        // 速度规划
        const auto local_speeds = local_speeds_planner_->cal_speed(decider_);
        if (local_speeds.local_speeds.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "local speeds is empty");
            return;
        }
        // 合成轨迹
        const auto local_trajectory = local_trajectory_combiner_->combine_local_trajectory(local_path, local_speeds);
        if (local_trajectory.local_trajectory.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "local trajectory is empty");
            return;
        }
        // local_path_pub_->publish(local_path_planner_->local_path_rviz());
        local_trajectory_pub_->publish(local_trajectory);

        // 更新绘图信息

        // 更新车辆信息
        car_->update_cartesian_info(local_trajectory.local_trajectory[0]); // 更新车辆的笛卡尔参数
        RCLCPP_INFO(this->get_logger(), "---------car state: loc: (%.2f, %.2f), speed: %.2f, a: %.2f, theta: %.2f, kappa: %.2f",
                    car_->loc_point().pose.position.x,
                    car_->loc_point().pose.position.y,
                    car_->speed(), car_->acceleration(),
                    car_->theta(), car_->kappa());
        const auto end_time = this->get_clock()->now();
        const double planning_total_time = end_time.seconds() - start_time.seconds();
        RCLCPP_INFO(this->get_logger(), "planning total time: %.2f ms", planning_total_time * 1000);

        // 防止系统法卡死
        if (planning_total_time > 1.0)
        {
            RCLCPP_ERROR(this->get_logger(), "planning time too long!");
            rclcpp::shutdown();
        }
    }
} // namespace Planning