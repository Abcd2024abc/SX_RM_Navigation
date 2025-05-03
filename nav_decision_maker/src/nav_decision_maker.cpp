#include "nav_decision_maker/nav_decision_maker.hpp"

NavDecisionMaker::NavDecisionMaker() : Node("nav_decision_maker") {
    // 设置目标点
    target_x = 4.5;
    target_y = -3.3;
    is_active_ = false;  // 默认不发送目标
    is_navigating_ = false;  // 默认不在导航中
    
    // 获取命名空间
    std::string namespace_ = this->get_namespace();
    if (namespace_.empty()) {
        namespace_ = "red_standard_robot1";  // 默认使用红色机器人
    }
    
    // 创建导航客户端
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, namespace_ + "/navigate_to_pose");
    RCLCPP_INFO(this->get_logger(), "创建导航客户端，命名空间: %s", namespace_.c_str());
    
    // 创建tf监听器
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // 创建服务
    set_target_service_ = this->create_service<SetTarget>(
        "set_target",
        std::bind(&NavDecisionMaker::handle_set_target, this, std::placeholders::_1, std::placeholders::_2));
    
    // 创建定时器，每1秒检查一次是否需要发送目标
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&NavDecisionMaker::send_goal, this));
        
    // 创建重试定时器
    retry_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&NavDecisionMaker::retry_connection, this));
        
    // 添加闪避相关参数
    enable_dodge_ = false;
    is_dodging_ = false;
    dodge_radius_MIN = 0.5;  // 闪避半径min0.5米
    dodge_radius_MAX = 0.5;  // 闪避半径max1米
    dodge_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),  // 每1秒生成一个新的闪避目标
        std::bind(&NavDecisionMaker::generate_dodge_goal, this));
    dodge_timer_->cancel();  // 初始时停止闪避定时器
    
    // 初始化随机数生成器
    std::random_device rd;
    rng_ = std::mt19937(rd());
    
    // 初始连接尝试
    retry_connection();
}

void NavDecisionMaker::retry_connection() {
    // 检查导航服务器连接
    if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "导航服务器未上线，5秒后重试...");
        return;
    }

    // 检查TF树
    try {
        auto transform = tf_buffer_->lookupTransform(
            "map", "base_footprint", 
            tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "无法获取机器人位置，5秒后重试: %s", ex.what());
    }
}

void NavDecisionMaker::handle_set_target(
    const std::shared_ptr<SetTarget::Request> request,
    std::shared_ptr<SetTarget::Response> response)
{
    // 更新目标点和闪避设置
    target_x = request->target_x;
    target_y = request->target_y;
    is_active_ = request->is_active;
    enable_dodge_ = request->enable_dodge;

    // 重置导航状态
    is_navigating_ = false;
    is_dodging_ = false;
    dodge_timer_->cancel();  // 停止闪避定时器

    RCLCPP_INFO(this->get_logger(), "重置导航状态");
    RCLCPP_INFO(this->get_logger(), "设置新目标点: x=%.2f, y=%.2f, 激活状态: %s, 闪避功能: %s",
        target_x, target_y, is_active_ ? "是" : "否", enable_dodge_ ? "开启" : "关闭");

    // 如果激活，立即发送目标
    if (is_active_) {
        send_goal();
    }

    response->success = true;
}

void NavDecisionMaker::generate_dodge_goal() {
    if (!is_dodging_) return;
    
    try {
        // 获取机器人当前位置
        auto transform = tf_buffer_->lookupTransform(
            "map", "base_footprint", 
            tf2::TimePointZero, 
            tf2::durationFromSec(5.0));
        
        // 计算当前位置到目标点的距离
        double dx = transform.transform.translation.x - target_x;
        double dy = transform.transform.translation.y - target_y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        // 如果距离超过闪避半径，停止闪避
        if (distance > dodge_radius_MAX) {
            RCLCPP_INFO(this->get_logger(), "超出闪避范围，返回目标点");
            // 创建返回目标点的位姿
            auto goal_pose = geometry_msgs::msg::PoseStamped();
            goal_pose.header.frame_id = "map";
            goal_pose.header.stamp = this->now();
            goal_pose.pose.position.x = target_x;
            goal_pose.pose.position.y = target_y;
            goal_pose.pose.position.z = 0.0;
            
            // 设置朝向（朝向目标点）
            double yaw = std::atan2(dy, dx);
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            goal_pose.pose.orientation.x = q.x();
            goal_pose.pose.orientation.y = q.y();
            goal_pose.pose.orientation.z = q.z();
            goal_pose.pose.orientation.w = q.w();
            
            // 发送返回目标
            auto goal_msg = NavigateToPose::Goal();
            goal_msg.pose = goal_pose;
            
            auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
            send_goal_options.goal_response_callback = [this](NavigationActionGoalHandle::SharedPtr goal_handle){
                if(!goal_handle){
                    RCLCPP_ERROR(this->get_logger(), "返回目标点请求失败");
                }
            };
            
            action_client_->async_send_goal(goal_msg, send_goal_options);
            return;
        }
        
        // 生成随机角度和距离
        std::uniform_real_distribution<double> angle_dist(0, 2*M_PI);
        std::uniform_real_distribution<double> dist_dist(dodge_radius_MIN, dodge_radius_MAX);  // 闪避半径MIN米到MAX米
        double angle = angle_dist(rng_);
        double dist = dist_dist(rng_);
        
        // 计算新的目标位置
        double new_x = transform.transform.translation.x + dist * std::cos(angle);
        double new_y = transform.transform.translation.y + dist * std::sin(angle);
        
        // 创建新的目标位姿
        auto goal_pose = geometry_msgs::msg::PoseStamped();
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = this->now();
        goal_pose.pose.position.x = new_x;
        goal_pose.pose.position.y = new_y;
        goal_pose.pose.position.z = 0.0;
        
        // 设置随机朝向
        double yaw = angle_dist(rng_);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        goal_pose.pose.orientation.x = q.x();
        goal_pose.pose.orientation.y = q.y();
        goal_pose.pose.orientation.z = q.z();
        goal_pose.pose.orientation.w = q.w();
        
        // 发送闪避目标
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal_pose;
        
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](NavigationActionGoalHandle::SharedPtr goal_handle){
            if(!goal_handle){
                RCLCPP_ERROR(this->get_logger(), "闪避目标请求失败");
            }
        };
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
        
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "无法获取机器人位置: %s", ex.what());
    }
}

void NavDecisionMaker::send_goal(){
    // 如果不激活或已经在导航中，不发送目标
    if (!is_active_ || is_navigating_) {
        return;
    }
    
    try {
        // 获取机器人当前位置
        auto transform = tf_buffer_->lookupTransform(
            "map", "base_footprint", 
            tf2::TimePointZero, 
            tf2::durationFromSec(5.0));
        
        // 使用节流日志输出位置信息
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
            "机器人当前位置: x=%.2f, y=%.2f", 
            transform.transform.translation.x, transform.transform.translation.y);
        
        //创建目标位姿
        auto goal_pose = geometry_msgs::msg::PoseStamped();
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = this->now();
        goal_pose.pose.position.x = target_x;
        goal_pose.pose.position.y = target_y;
        goal_pose.pose.position.z = 0.0;  // 设置z坐标为0
        
        // 设置朝向（朝向目标点）
        double dx = target_x - transform.transform.translation.x;
        double dy = target_y - transform.transform.translation.y;
        double yaw = std::atan2(dy, dx);
        
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        goal_pose.pose.orientation.x = q.x();
        goal_pose.pose.orientation.y = q.y();
        goal_pose.pose.orientation.z = q.z();
        goal_pose.pose.orientation.w = q.w();

        // 使用节流日志输出目标信息
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "发送导航目标: x=%.2f, y=%.2f, yaw=%.2f", 
            target_x, target_y, yaw);

        //创建目标消息
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal_pose;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        //请求回调函数
        send_goal_options.goal_response_callback = [this](NavigationActionGoalHandle::SharedPtr goal_handle){
            if(!goal_handle){
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "目标位置请求失败，5秒后重试");
                // 启动重试定时器
                retry_timer_->reset();
                is_navigating_ = false;
            }else{
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "目标位置请求成功");
                is_navigating_ = true;
            }
        };
        
        //移动回调函数
        send_goal_options.feedback_callback = [this](NavigationActionGoalHandle::SharedPtr goal_handle,const std::shared_ptr<const NavigateToPose::Feedback> feedback){
            (void)goal_handle;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "剩余距离:%.2f 米",feedback->distance_remaining);
        };
        
        //结果回调函数
        send_goal_options.result_callback = [this](const NavigationActionGoalHandle::WrappedResult& result) {
            is_navigating_ = false;  // 导航结束，重置状态
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED: {
                    RCLCPP_INFO(this->get_logger(), "导航成功");

                    // 检查机器人是否在目标点附近
                    try {
                        auto transform = tf_buffer_->lookupTransform(
                            "map", "base_footprint", 
                            tf2::TimePointZero, 
                            tf2::durationFromSec(1.0));

                        double dx = transform.transform.translation.x - target_x;
                        double dy = transform.transform.translation.y - target_y;
                        double distance = std::sqrt(dx * dx + dy * dy);

                        if (distance <= dodge_radius_MAX) {
                            RCLCPP_INFO(this->get_logger(), "机器人已到达目标点附近，偏移距离: %.2f 米", distance);

                            // 如果启用了闪避功能，开始闪避
                            if (enable_dodge_) {
                                is_dodging_ = true;
                                dodge_timer_->reset();
                                RCLCPP_INFO(this->get_logger(), "开始闪避模式");
                            }
                        } else {
                            RCLCPP_WARN(this->get_logger(), "机器人未到达目标点，偏移距离: %.2f 米", distance);
                        }
                    } catch (const tf2::TransformException &ex) {
                        RCLCPP_ERROR(this->get_logger(), "无法获取机器人位置: %s", ex.what());
                    }

                    is_active_ = false;  // 导航成功后，停止发送目标
                    break;
                }
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "导航失败，可能原因：目标点不可达或路径规划失败");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "导航取消");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "导航结果异常");
                    break;
            }
        };
        
        //发送目标位置
        action_client_->async_send_goal(goal_msg, send_goal_options);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "无法获取机器人位置，1秒后重试: %s", ex.what());
        // 启动重试定时器
        retry_timer_->reset();
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto nav_decision_maker = std::make_shared<NavDecisionMaker>();
    rclcpp::spin(nav_decision_maker);
    rclcpp::shutdown();
    return 0;
}