#include "rclcpp/rclcpp.hpp"
#include <math.h>
#include <iostream>
#include <Eigen/Dense>


#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

// 使用正確的頭文件名稱
#include <costmap_converter_msgs/msg/obstacle_msg.hpp>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/empty.hpp" //!!!!!!!!!!
#include "campusrover_msgs/srv/planner_function.hpp"
#include "campusrover_msgs/srv/elevator_status_checker.hpp"
#include "campusrover_move/srv/pull_over_path_generator.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846  /* pi */
#endif

using namespace std::chrono_literals;  // 用於時間文字表示，例如：100ms
using std::placeholders::_1;           // 用於綁定回調函數
using std::placeholders::_2;           // 用於綁定回調函數

/**
 * @brief 路徑跟隨節點類
 * 
 * 負責接收路徑，跟隨路徑，避障等功能
 */
class PathFollowing : public rclcpp::Node
{
public:
    // 構造函數
    PathFollowing();

private:
    // 訂閱者
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr elevator_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr custom_obst_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr click_sub_;

    // 發布者
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr twist_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr global_status_check_pub_;

    // 服務客戶端
    rclcpp::Client<campusrover_msgs::srv::ElevatorStatusChecker>::SharedPtr elevator_status_check_client_;
    rclcpp::Client<campusrover_msgs::srv::PlannerFunction>::SharedPtr dwa_planner_client_;
    // rclcpp::Client<campusrover_move::srv::PullOverPathGenerator>::SharedPtr pullover_planner_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr global_status_check_client_; //!!!!!!!!!!
    // 服務服務器
    rclcpp::Service<campusrover_msgs::srv::PlannerFunction>::SharedPtr planner_function_service_;

    // 定時器
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr msgs_timer_;

    // TF2 相關
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 存儲路徑和位姿等數據
    nav_msgs::msg::Path following_path_;
    nav_msgs::msg::Path globle_path_; 
    nav_msgs::msg::Path elevator_path_; 
    nav_msgs::msg::Path pullover_path_;
    geometry_msgs::msg::PoseStamped target_pose_;
    nav_msgs::msg::OccupancyGrid costmap_data_;
    geometry_msgs::msg::Pose robot_tf_pose_;
    geometry_msgs::msg::PoseArray obstacle_poses_;

    // 用於服務請求的消息
    std_msgs::msg::Empty global_status_checker_msg_;

    // 啟動時間
    rclcpp::Time planner_start_time_;

    // 參數
    std::string robot_frame_;
int status_msg_;
int path_sub_mode_;
double arriving_range_dis_;
double arriving_range_angle_;
bool action_flag_ = false;
bool get_following_path_ = false;
bool get_costmap_data_ = false;
bool get_obstacle_data_ = false;
bool obstacle_stop_cmd_ = false;
bool arriving_end_point_= false;
bool arriving_end_direction_= false;
bool enble_costmap_obstacle_;
bool direction_inverse_= false;
bool get_velocity_data_ = false;
bool call_dwa_enable_srv_ = false;
bool call_dwa_disable_srv_ = true;
bool enable_linear_depend_angular_;
bool enble_dwa_obstacle_avoidance_;
    // bool enble_pullover_mode_;
double max_angle_of_linear_profile_;
double min_angle_of_linear_profile_;
double threshold_occupied_;
double footprint_max_x_;
double footprint_min_x_;
double footprint_max_y_;
double footprint_min_y_;
double robot_yaw_;
double speed_pid_k_;
double target_yaw_;
double max_linear_velocity_;
double min_linear_velocity_;
double max_angular_velocity_;
double min_angular_velocity_;
double target_point_dis_;
double obstacle_detect_max_dis_;
double obstacle_detect_min_dis_;
double back_obstacle_detect_max_dis_;
double obstacle_range_;
double active_angular_;
double twist_linear_step_ = 0;
double twist_angular_step_ = 0;
double current_v_;
double current_w_;

    // 回調函數
    void timer_callback();
    void msgs_timer_callback();
    void elevator_path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void global_path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void custom_obstacle_callback(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr msg);
    void click_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void service_callback(
        const std::shared_ptr<campusrover_msgs::srv::PlannerFunction::Request> request,
        const std::shared_ptr<campusrover_msgs::srv::PlannerFunction::Response> response);

    // 工具函數
    void get_parameters();
    void update_campusrover_pose_from_tf();
    void check_arrive_point();
    void check_arrive_direction();
    void moving_to_target_point();
    void moving_to_target_direction();
    void angle_normalize(double &angle);
    void elevator_status_check_call_service(
        const std::shared_ptr<campusrover_msgs::srv::ElevatorStatusChecker::Request> request);
    void twist_publish(double x, double z);
    void twist_profile(double &profile_linear_x, double &profile_angular_z);
    void update_input_path(const nav_msgs::msg::Path &path);
    // void pullover_function();
};

/**
 * @brief 主函數
 * 
 * 初始化 ROS2 節點並創建 PathFollowing 對象
 */
int main(int argc, char **argv)
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);
    
    // 創建節點並自動處理
    auto node = std::make_shared<PathFollowing>();
    
    // 處理回調
    rclcpp::spin(node);
    
    // 關閉 ROS2
    rclcpp::shutdown();
    
    return 0;
}

/**
 * @brief 構造函數
 * 
 * 初始化參數，創建發布者、訂閱者和服務
 */
PathFollowing::PathFollowing() : Node("path_following")
{
    // 初始化 TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 獲取參數
    get_parameters();

    // 創建發布者
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 50);
    twist_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("twist_path", 20);
    path_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("local_trajectories", 10);
    global_status_check_pub_ = this->create_publisher<std_msgs::msg::Empty>("reach_goal", 20);

    // 創建訂閱者
    elevator_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "elevator_path", 10, std::bind(&PathFollowing::elevator_path_callback, this, _1));
    
    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "global_path", 10, std::bind(&PathFollowing::global_path_callback, this, _1));
    
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "costmap", 10, std::bind(&PathFollowing::costmap_callback, this, _1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&PathFollowing::odom_callback, this, _1));
    
    click_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&PathFollowing::click_callback, this, _1));

    custom_obst_sub_ = this->create_subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>(
        "obstacles", 1, std::bind(&PathFollowing::custom_obstacle_callback, this, _1));

    // 創建服務服務器
    planner_function_service_ = this->create_service<campusrover_msgs::srv::PlannerFunction>(
        "planner_function", std::bind(&PathFollowing::service_callback, this, _1, _2));

    // 創建服務客戶端
    elevator_status_check_client_ = this->create_client<campusrover_msgs::srv::ElevatorStatusChecker>("elevator_status_checker");
    dwa_planner_client_ = this->create_client<campusrover_msgs::srv::PlannerFunction>("planner_function_dwa");
    // pullover_planner_client_ = this->create_client<campusrover_move::srv::PullOverPathGenerator>("generate_pullover_path");
    global_status_check_client_ = this->create_client<std_srvs::srv::Empty>("global_status_check"); //!!!!!!!!!!

    // 創建定時器
    timer_ = this->create_wall_timer(20ms, std::bind(&PathFollowing::timer_callback, this));
    msgs_timer_ = this->create_wall_timer(2s, std::bind(&PathFollowing::msgs_timer_callback, this));

    // 初始化時間
    planner_start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Path Following Node Initialized");
}

/**
 * @brief 獲取參數
 * 
 * 從參數服務器中獲取所有需要的參數
 */
void PathFollowing::get_parameters()
{
    // 聲明參數，設置默認值
    this->declare_parameter("robot_frame", "base_link");
    this->declare_parameter("arriving_range_dis", 0.2);
    this->declare_parameter("arriving_range_angle", 0.1);

    this->declare_parameter("max_linear_velocity", 0.5);
    this->declare_parameter("min_linear_velocity", 0.05);
    this->declare_parameter("max_angular_velocity", 0.3);
    this->declare_parameter("min_angular_velocity", 0.05);
    this->declare_parameter("target_point_dis", 0.5);

    this->declare_parameter("threshold_occupied", 10.0);
    this->declare_parameter("footprint_max_x", 1.5);
    this->declare_parameter("footprint_min_x", -0.5);
    this->declare_parameter("footprint_max_y", 0.5);
    this->declare_parameter("footprint_min_y", -0.5);
    this->declare_parameter("obstacle_range", 0.3);
    this->declare_parameter("obstacle_detect_max_dis", 1.5);
    this->declare_parameter("obstacle_detect_min_dis", 0.8);
    this->declare_parameter("back_obstacle_detect_max_dis", 2.3);
    this->declare_parameter("speed_pid_k", 0.06);

    this->declare_parameter("min_angle_of_linear_profile", 0.1);
    this->declare_parameter("max_angle_of_linear_profile", 0.5);

    this->declare_parameter("enble_costmap_obstacle", false);
    this->declare_parameter("enble_dwa_obstacle_avoidance", false);
    // this->declare_parameter("enble_pullover_mode", false);
    this->declare_parameter("direction_inverse", false);
    this->declare_parameter("enable_linear_depend_angular", false);

    // 獲取參數值
    robot_frame_ = this->get_parameter("robot_frame").as_string();
    arriving_range_dis_ = this->get_parameter("arriving_range_dis").as_double();
    arriving_range_angle_ = this->get_parameter("arriving_range_angle").as_double();
    max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
    min_linear_velocity_ = this->get_parameter("min_linear_velocity").as_double();
    max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
    min_angular_velocity_ = this->get_parameter("min_angular_velocity").as_double();
    target_point_dis_ = this->get_parameter("target_point_dis").as_double();
    threshold_occupied_ = this->get_parameter("threshold_occupied").as_double();
    footprint_max_x_ = this->get_parameter("footprint_max_x").as_double();
    footprint_min_x_ = this->get_parameter("footprint_min_x").as_double();
    footprint_max_y_ = this->get_parameter("footprint_max_y").as_double();
    footprint_min_y_ = this->get_parameter("footprint_min_y").as_double();
    obstacle_range_ = this->get_parameter("obstacle_range").as_double();
    obstacle_detect_max_dis_ = this->get_parameter("obstacle_detect_max_dis").as_double();
    obstacle_detect_min_dis_ = this->get_parameter("obstacle_detect_min_dis").as_double();
    back_obstacle_detect_max_dis_ = this->get_parameter("back_obstacle_detect_max_dis").as_double();
    speed_pid_k_ = this->get_parameter("speed_pid_k").as_double();
    min_angle_of_linear_profile_ = this->get_parameter("min_angle_of_linear_profile").as_double();
    max_angle_of_linear_profile_ = this->get_parameter("max_angle_of_linear_profile").as_double();
    enble_costmap_obstacle_ = this->get_parameter("enble_costmap_obstacle").as_bool();
    enble_dwa_obstacle_avoidance_ = this->get_parameter("enble_dwa_obstacle_avoidance").as_bool();
    // enble_pullover_mode_ = this->get_parameter("enble_pullover_mode").as_bool();
    direction_inverse_ = this->get_parameter("direction_inverse").as_bool();
    enable_linear_depend_angular_ = this->get_parameter("enable_linear_depend_angular").as_bool();
    
    // 記錄參數
    RCLCPP_INFO(this->get_logger(), "Parameters loaded: robot_frame=%s", robot_frame_.c_str());
}

/**
 * @brief 處理電梯路徑訂閱的回調
 * 
 * @param msg 接收到的路徑消息
 */
void PathFollowing::elevator_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{ 
    // 檢查是否在電梯路徑模式
    if(path_sub_mode_ != campusrover_msgs::srv::PlannerFunction::Request::MODE_ELEVATOR_PATH)
    {
        return;
    }

    // 檢查路徑是否為空
    if(msg->poses.size() == 0)
    {
        return;
    }

    // 保存電梯路徑
    elevator_path_.header.frame_id = msg->header.frame_id;
    elevator_path_.poses.clear();
    elevator_path_ = *msg;

    // 更新當前跟隨的路徑
    update_input_path(elevator_path_);
    
    RCLCPP_INFO(this->get_logger(), "Received elevator path with %zu points", msg->poses.size());
}

/**
 * @brief 處理全局路徑訂閱的回調
 * 
 * @param msg 接收到的路徑消息
 */
void PathFollowing::global_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    // 檢查路徑是否為空
    if(msg->poses.size() == 0)
        return;

    // 保存全局路徑
    globle_path_.header.frame_id = msg->header.frame_id;
    globle_path_.poses.clear();
    globle_path_ = *msg;

    // 如果當前是全局路徑模式，則更新當前跟隨的路徑
    if(path_sub_mode_ == campusrover_msgs::srv::PlannerFunction::Request::MODE_GLOBAL_PATH)
    {
        update_input_path(globle_path_);
    }
    
    RCLCPP_INFO(this->get_logger(), "Received global path with %zu points", msg->poses.size());
}

/**
 * @brief 處理代價地圖訂閱的回調
 * 
 * @param msg 接收到的代價地圖消息
 */
void PathFollowing::costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    // 如果未啟用代價地圖障礙物檢測，則直接返回
    if(!enble_costmap_obstacle_)
        return;

    geometry_msgs::msg::Pose ob_pose;
    geometry_msgs::msg::PoseStamped ob_posestamped;
    geometry_msgs::msg::PoseStamped base_ob_pose;
    double data;
    double value;

    // 保存代價地圖分辨率
    costmap_data_.info.resolution = msg->info.resolution;
    obstacle_poses_.poses.clear();

    // 如果代價地圖參考框架與機器人框架相同
    if (msg->header.frame_id == robot_frame_)
    {
        for (size_t i = 0; i < msg->data.size(); i++)
        {
            data = static_cast<double>(msg->data[i]);
            value = i;

            // 如果該單元格被佔用
            if (std::abs(data) > threshold_occupied_)
            {
                // 計算障礙物在地圖中的位置
                ob_pose.position.y = (std::floor(value / msg->info.width) * msg->info.resolution) + msg->info.origin.position.y;
                ob_pose.position.x = ((value - (msg->info.width * std::floor(value / msg->info.width))) * msg->info.resolution) + msg->info.origin.position.x;

                // 檢查障礙物是否在機器人足跡內
                if (ob_pose.position.x < footprint_max_x_ && ob_pose.position.x > footprint_min_x_ && 
                    ob_pose.position.y < footprint_max_y_ && ob_pose.position.y > footprint_min_y_)
                {
                    obstacle_stop_cmd_ = true;
                    status_msg_ = 3;
                    break;
  }
  else
  {
                    obstacle_stop_cmd_ = false;
                }
                obstacle_poses_.poses.push_back(ob_pose);
            }
        }
    }
    else
    {
        // 如果代價地圖參考框架不同於機器人框架，需要進行坐標轉換
        ob_posestamped.header.frame_id = msg->header.frame_id;

        for (size_t i = 0; i < msg->data.size(); i++)
        {
            data = static_cast<double>(msg->data[i]);

            if (std::abs(data) > threshold_occupied_)
            {
                value = i;

                // 計算障礙物在地圖坐標系中的位置
                ob_posestamped.pose.position.y = (std::floor(value / msg->info.width) * msg->info.resolution) + msg->info.origin.position.y;
                ob_posestamped.pose.position.x = ((value - (msg->info.width * std::floor(value / msg->info.width))) * msg->info.resolution) + msg->info.origin.position.x;

                try
                {
                    // 將障礙物位置轉換到機器人坐標系
                    tf_buffer_->transform(ob_posestamped, base_ob_pose, robot_frame_, tf2::durationFromSec(2.0));
                }
                catch (tf2::TransformException &ex)
                {
                    RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
                    return;
                }

                ob_pose.position.y = base_ob_pose.pose.position.y;
                ob_pose.position.x = base_ob_pose.pose.position.x;

                // 檢查障礙物是否在機器人足跡內
                if (ob_pose.position.x < footprint_max_x_ && ob_pose.position.x > footprint_min_x_ && 
                    ob_pose.position.y < footprint_max_y_ && ob_pose.position.y > footprint_min_y_)
                {
                    obstacle_stop_cmd_ = true;
                    status_msg_ = 3;
                    break;
                }
                else
                {
                    obstacle_stop_cmd_ = false;
                }
                obstacle_poses_.poses.push_back(ob_pose);
            }
        }
    }

    get_costmap_data_ = true;
}

/**
 * @brief 處理里程計訂閱的回調
 * 
 * @param msg 接收到的里程計消息
 */
void PathFollowing::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // 保存當前線速度和角速度
    current_v_ = msg->twist.twist.linear.x;
    current_w_ = msg->twist.twist.angular.z;

    // 如果是第一次獲取速度數據，打印日誌
    if(get_velocity_data_ == false){
        RCLCPP_INFO(this->get_logger(), "Path following: Got odometry data!");
    }
    get_velocity_data_ = true;
}

/**
 * @brief 處理自定義障礙物訂閱的回調
 * 
 * @param msg 接收到的障礙物消息
 */
void PathFollowing::custom_obstacle_callback(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
{
    geometry_msgs::msg::Pose ob_pose;
    geometry_msgs::msg::PoseStamped obstacle_r;
    geometry_msgs::msg::PoseStamped obstacle_g;

    obstacle_poses_.poses.clear();

    // 如果障礙物坐標系與機器人坐標系相同
    if (msg->header.frame_id == robot_frame_)
    {
        for(size_t i=0; i < msg->obstacles.size(); i++)
        {
            ob_pose.position.x = msg->obstacles[i].polygon.points[0].x;
            ob_pose.position.y = msg->obstacles[i].polygon.points[0].y;    

            obstacle_poses_.poses.push_back(ob_pose);
        }
    }
    else
    {
        // 如果障礙物坐標系與機器人坐標系不同，需要進行坐標轉換
        for(size_t i=0; i < msg->obstacles.size(); i++)
        {
            obstacle_g.header.frame_id = msg->obstacles[i].header.frame_id;
            obstacle_g.pose.position.x = msg->obstacles[i].polygon.points[0].x;
            obstacle_g.pose.position.y = msg->obstacles[i].polygon.points[0].y;
            try
            {
                tf_buffer_->transform(obstacle_g, obstacle_r, robot_frame_, tf2::durationFromSec(2.0));
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
                return;
            }

            ob_pose.position.x = obstacle_r.pose.position.x;
            ob_pose.position.y = obstacle_r.pose.position.y;    

            obstacle_poses_.poses.push_back(ob_pose);
        }
    }

    get_obstacle_data_ = true;
}

/**
 * @brief 處理規劃器功能服務的回調
 * 
 * @param request 請求
 * @param response 響應
 */
void PathFollowing::service_callback(
    const std::shared_ptr<campusrover_msgs::srv::PlannerFunction::Request> request,
    const std::shared_ptr<campusrover_msgs::srv::PlannerFunction::Response> response)
{
    static int last_mode;
    
    // 更新參數
    action_flag_ = request->action.data;
    direction_inverse_ = request->direction_inverse.data;
    enble_costmap_obstacle_ = request->obstacle_avoidance.data;
    max_linear_velocity_ = request->speed_parameter.linear.x;
    max_angular_velocity_ = std::abs(request->speed_parameter.angular.z);
    active_angular_ = request->speed_parameter.angular.z;
    path_sub_mode_ = request->mode;

    RCLCPP_INFO(this->get_logger(), "Received planner function request:");
    RCLCPP_INFO(this->get_logger(), "  action_flag: %s", action_flag_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  direction_inverse: %s", direction_inverse_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  enable_costmap_obstacle: %s", enble_costmap_obstacle_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  max_linear_velocity: %.2f", max_linear_velocity_);
    RCLCPP_INFO(this->get_logger(), "  max_angular_velocity: %.2f", max_angular_velocity_);
    
    // 如果停止動作或者模式變化，重置狀態
    if(!action_flag_ || path_sub_mode_ != last_mode)
    {
        arriving_end_point_ = false;
    arriving_end_direction_ = false;
        get_costmap_data_ = false;
        twist_linear_step_ = 0;
        twist_angular_step_ = 0;
    }

    planner_start_time_ = this->now();

    // 根據模式更新當前跟隨的路徑
    if(path_sub_mode_ == campusrover_msgs::srv::PlannerFunction::Request::MODE_GLOBAL_PATH)
    {
        update_input_path(globle_path_);
    }
    else if(path_sub_mode_ == campusrover_msgs::srv::PlannerFunction::Request::MODE_ELEVATOR_PATH)
    {
        update_input_path(elevator_path_);
    }
    // else if(path_sub_mode_ == campusrover_msgs::srv::PlannerFunction::Request::MODE_PULLOVER_PATH)
    // {
    //     update_input_path(pullover_path_);
    // }

    // 如果不使用代價地圖障礙物檢測，清空障礙物數組
    if(!enble_costmap_obstacle_)
    {
        obstacle_poses_.poses.clear();
    }
    
    last_mode = path_sub_mode_;
}

/**
 * @brief 主定時器回調，執行路徑跟隨
 */
void PathFollowing::timer_callback()
{
    // 如果未啟用動作，不執行
  if(!action_flag_)
    return;
    
    // 如果是按鈕停車模式
    if(path_sub_mode_ == campusrover_msgs::srv::PlannerFunction::Request::MODE_BUTTON_PARKING)
    {
        twist_publish(max_linear_velocity_, active_angular_);
  }
  else
  {
        // 如果有路徑可以跟隨
    if (get_following_path_)
    {
            // 如果有代價地圖數據或障礙物數據，或者不需要檢測障礙物
            if (get_costmap_data_ || get_obstacle_data_ || !enble_costmap_obstacle_)
            {
                // 更新機器人當前位姿
                update_campusrover_pose_from_tf();
                
                // 如果啟用了避讓模式且未啟用DWA避障
                // if(enble_pullover_mode_ && !call_dwa_enable_srv_)
                // {
                //     pullover_function();
                // }
                
                // 如果還未到達正確方向
        if(!arriving_end_direction_)
        {
                    // 如果還未到達目標點
          if(!arriving_end_point_)
          {
            check_arrive_point();
            if(arriving_end_point_)
            {
              moving_to_target_direction();
              return;
            }
            moving_to_target_point();
          }
          else
          {
                        // 已到達目標點，但未到達正確方向
            check_arrive_direction();
            if(arriving_end_direction_)
            {
              status_msg_ = 4;
                            twist_publish(0.0, 0.0);
              return;
            }

            moving_to_target_direction();
          }
        }
        else
        {
                    // 已到達正確方向，檢查是否到達目標點
          check_arrive_point();
          status_msg_ = 4;
                    twist_publish(0.0, 0.0);
        }
      }
      else
      {
                // 沒有代價地圖數據或障礙物數據
        status_msg_ = 2;
                twist_publish(0.0, 0.0);
      }
    }
    else
    {
            // 沒有路徑可以跟隨
      status_msg_ = 1;
            twist_publish(0.0, 0.0);
        }
    }
}

/**
 * @brief 消息定時器回調，用於打印狀態信息
 */
void PathFollowing::msgs_timer_callback()
{
    // 根據狀態碼打印相應信息
  if (status_msg_ == 1)
  {
        RCLCPP_WARN(this->get_logger(), "Path following: No path to follow, waiting for path input");
    status_msg_ = 0;
  }
  else if (status_msg_ == 2)
  {
        RCLCPP_WARN(this->get_logger(), "Path following: No costmap data, waiting for costmap input");
    status_msg_ = 0;
  }
  else if (status_msg_ == 3)
  {
        RCLCPP_INFO(this->get_logger(), "Path following: Detected obstacle");
    status_msg_ = 0;
  }
  else if (status_msg_ == 4)
  {
        RCLCPP_INFO(this->get_logger(), "Path following: Arrived at destination");
    status_msg_ = 0;
  }
  else
  {
    status_msg_ = 0;
    }
}

/**
 * @brief 從 TF 更新機器人位姿
 */
void PathFollowing::update_campusrover_pose_from_tf()
{
    geometry_msgs::msg::TransformStamped transformStamped;
    double roll, pitch, yaw;
    double pre_yaw;

    try
    {
        // 查詢從地圖到機器人的變換
        transformStamped = tf_buffer_->lookupTransform(
            following_path_.header.frame_id, robot_frame_, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "%s. Can't update pose from TF, will use the latest source point.", ex.what());
        return;
    }

    // 更新機器人位姿
    robot_tf_pose_.position.x = transformStamped.transform.translation.x;
    robot_tf_pose_.position.y = transformStamped.transform.translation.y;
    robot_tf_pose_.position.z = transformStamped.transform.translation.z;
    robot_tf_pose_.orientation = transformStamped.transform.rotation;

    // 從四元數獲取歐拉角
    tf2::Quaternion q(
        robot_tf_pose_.orientation.x,
        robot_tf_pose_.orientation.y,
        robot_tf_pose_.orientation.z,
        robot_tf_pose_.orientation.w);
    tf2::Matrix3x3 m(q);
    
    m.getRPY(roll, pitch, yaw);
    
    // 根據是否反向駕駛調整航向角
    if(!direction_inverse_)
    {
        pre_yaw = yaw;
    }
    else
    {
        pre_yaw = yaw + M_PI;
        angle_normalize(pre_yaw);
    }
    robot_yaw_ = pre_yaw;
}

/**
 * @brief 檢查是否到達目標點
 */
void PathFollowing::check_arrive_point()
{
    // 計算機器人到目標點的距離
    double dist = std::sqrt(std::pow(target_pose_.pose.position.x - robot_tf_pose_.position.x, 2) +
                         std::pow(target_pose_.pose.position.y - robot_tf_pose_.position.y, 2));

    // 判斷是否到達目標點
    if(dist < arriving_range_dis_)
    {
        arriving_end_point_ = true;
        twist_linear_step_ = 0;
    }
    else
    {
        arriving_end_point_ = false;
        arriving_end_direction_ = false;
    }
}

/**
 * @brief 檢查是否到達目標方向
 */
void PathFollowing::check_arrive_direction()
{
    // 計算方向誤差
    double angle_error = target_yaw_ - robot_yaw_;
    angle_normalize(angle_error);

    // 判斷是否到達目標方向
    if(std::abs(angle_error) < arriving_range_angle_)
    {
        arriving_end_direction_ = true;
        
        // 根據不同模式處理到達目標方向的事件
        if(path_sub_mode_ == campusrover_msgs::srv::PlannerFunction::Request::MODE_ELEVATOR_PATH)
        {
            // 創建電梯狀態檢查請求
            auto request = std::make_shared<campusrover_msgs::srv::ElevatorStatusChecker::Request>();
            request->node_name.data = "planner";
            request->status.data = arriving_end_direction_;
            
            // 發送請求
            elevator_status_check_call_service(request);
        }
        else if(path_sub_mode_ == campusrover_msgs::srv::PlannerFunction::Request::MODE_GLOBAL_PATH)
        {
            // 發布到達目標的消息
            global_status_check_pub_->publish(global_status_checker_msg_);
        }
        
        twist_angular_step_ = 0;
    }
    else
    {
        arriving_end_direction_ = false;
    }
}

/**
 * @brief 移動到目標點
 */
void PathFollowing::moving_to_target_point()
{
    double x_p, y_p;
    double dist_fp_p;
    double closest_dist;
    int closest_id;
    double looking_dist;
    double ob_dist;
    double front_ob_closest_dis;
    int target_point_id;
    int front_ob_detect_id;
    int back_ob_detect_id;
    geometry_msgs::msg::PoseStamped target_pose;

    // 查找最近的路徑點
    for (size_t cp = 0; cp < following_path_.poses.size(); cp++)
  {
    x_p = following_path_.poses[cp].pose.position.x;
    y_p = following_path_.poses[cp].pose.position.y;
        dist_fp_p = std::sqrt(std::pow(robot_tf_pose_.position.x - x_p, 2) + 
                           std::pow(robot_tf_pose_.position.y - y_p, 2));
    if (cp == 0)
    {
      closest_dist = dist_fp_p;
      closest_id = cp;
    }
    else if (dist_fp_p < closest_dist)
    {
      closest_dist = dist_fp_p;
      closest_id = cp;
    }
  }
  
  target_point_id = closest_id;
  front_ob_detect_id = closest_id;
  back_ob_detect_id = closest_id;
    
    // 查找在目標距離內的前方點
    if (closest_id < static_cast<int>(following_path_.poses.size()) - 1)
  {
        for(int fp = closest_id; fp < static_cast<int>(following_path_.poses.size()); fp++)
    {
            looking_dist = std::sqrt(std::pow(following_path_.poses[fp].pose.position.x - robot_tf_pose_.position.x, 2) + 
                                  std::pow(following_path_.poses[fp].pose.position.y - robot_tf_pose_.position.y, 2));

      if(looking_dist <= target_point_dis_)
      {
        target_point_id = fp;
      }

      if(looking_dist <= obstacle_detect_max_dis_)
      {
        front_ob_detect_id = fp;
      }
      else
      {
        break;
      }
    }
  }

    // 設置目標點
  target_pose.header.frame_id = following_path_.header.frame_id;
  target_pose.pose.position.x = following_path_.poses[target_point_id].pose.position.x;
  target_pose.pose.position.y = following_path_.poses[target_point_id].pose.position.y;
  target_pose.pose.position.z = following_path_.poses[target_point_id].pose.position.z;

    geometry_msgs::msg::PoseStamped following_path_pose_g;
    geometry_msgs::msg::PoseStamped following_path_pose_b;

    // 障礙物檢測相關變量
    static rclcpp::Time detect_static_ob_first_time;
    static rclcpp::Time no_detect_ob_first_time;
  double detect_ob_continue_time;
  double no_detect_ob_continue_time;

  bool detect_front_obstacle = false;
  static bool detect_front_obstacled = false;
  static bool detect_static_obstacled = false;

    // 搜索前方障礙物
    for(int path_count = closest_id; path_count < front_ob_detect_id; path_count++)
  {
    if(detect_front_obstacle){
      break;
    }
        
    following_path_pose_g.header.frame_id = following_path_.header.frame_id;
    following_path_pose_g.pose.position.x = following_path_.poses[path_count].pose.position.x;
    following_path_pose_g.pose.position.y = following_path_.poses[path_count].pose.position.y;
    following_path_pose_g.pose.position.z = following_path_.poses[path_count].pose.position.z;
        
    try{
            tf_buffer_->transform(following_path_pose_g, following_path_pose_b, robot_frame_, tf2::durationFromSec(2.0));
    }
    catch (tf2::TransformException &ex){
            RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
      return;
    }
        
        // 檢查每個障礙物
        for(size_t ob_count = 0; ob_count < obstacle_poses_.poses.size(); ob_count++)
    {
            ob_dist = std::sqrt(std::pow(following_path_pose_b.pose.position.x - obstacle_poses_.poses[ob_count].position.x, 2) + 
                             std::pow(following_path_pose_b.pose.position.y - obstacle_poses_.poses[ob_count].position.y, 2));
      
      if(ob_dist < obstacle_range_){
                front_ob_closest_dis = std::sqrt(std::pow(following_path_pose_b.pose.position.x, 2) + 
                                              std::pow(following_path_pose_b.pose.position.y, 2));
        
        detect_front_obstacle = true;
        break;
      }
    }
  }

    // 計算方向和速度
    double direction_yaw = std::atan2(target_pose.pose.position.y - robot_tf_pose_.position.y, 
                                   target_pose.pose.position.x - robot_tf_pose_.position.x);
    
    double yaw_error = direction_yaw - robot_yaw_;
  angle_normalize(yaw_error);

    double ang_vel = yaw_error * speed_pid_k_;
    double len_vel;

    // 根據行駛方向設置線速度
  if(direction_inverse_)
  {
    len_vel = -max_linear_velocity_;
  }
  else
  {
    len_vel = max_linear_velocity_;
  }

    // DWA 避障
  if(enble_dwa_obstacle_avoidance_)
  {
        // 檢測到前方障礙物
    if(detect_front_obstacle)
    {
      if(!detect_front_obstacled)
      {
        detect_front_obstacled = true;
      }

            // 檢查障礙物是否靜止
            if((front_ob_closest_dis - obstacle_detect_min_dis_) / (obstacle_detect_max_dis_ - obstacle_detect_min_dis_) < 0.1)
      {
        if(!detect_static_obstacled)
        {
                    detect_static_ob_first_time = this->now();
          detect_static_obstacled = true;
        }
                detect_ob_continue_time = (this->now() - detect_static_ob_first_time).seconds();
      }
      else
      {
        if(detect_static_obstacled)
        {
          detect_static_obstacled = false;
        }
      }
      
            // 如果靜止障礙物檢測時間超過閾值，啟用 DWA 規劃
      if(detect_static_obstacled && detect_ob_continue_time > 4.0)
      {
        if(!call_dwa_enable_srv_)
        {
                    // 創建 DWA 規劃請求
                    auto request = std::make_shared<campusrover_msgs::srv::PlannerFunction::Request>();
                    request->mode = campusrover_msgs::srv::PlannerFunction::Request::MODE_GLOBAL_PATH;
                    request->action.data = true;
                    request->obstacle_avoidance.data = true;
                    request->direction_inverse.data = false;
                    request->speed_parameter.linear.x = 0.4;
                    request->speed_parameter.angular.z = 0.4;
                    
                    // 發送請求
                    auto future = dwa_planner_client_->async_send_request(request);
                    
                    // 等待響應
                    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
                        rclcpp::FutureReturnCode::SUCCESS)
                    {
                        RCLCPP_INFO(this->get_logger(), "Started DWA Planner");
          }
          else
          {
                        RCLCPP_ERROR(this->get_logger(), "Failed to call DWA Planner Service");
            return;
          }
                    
          call_dwa_enable_srv_ = true;
          call_dwa_disable_srv_ = false;
        }
        return;
      }
      
            // 根據障礙物距離調整速度
      if(front_ob_closest_dis <= obstacle_detect_min_dis_)
      {
        len_vel = 0.0;
      }
      else
      {
                len_vel = len_vel * (front_ob_closest_dis - obstacle_detect_min_dis_) / 
                          (obstacle_detect_max_dis_ - obstacle_detect_min_dis_);
      }
    }
    else
    {
            // 未檢測到障礙物
      if(detect_front_obstacled)
      {
                no_detect_ob_first_time = this->now();
        detect_front_obstacled = false;
        detect_static_obstacled = false;
      }

            no_detect_ob_continue_time = (this->now() - no_detect_ob_first_time).seconds();
            
            // 如果一段時間沒有檢測到障礙物，禁用 DWA 規劃
      if(no_detect_ob_continue_time > 4.0)
      {
        if(!call_dwa_disable_srv_)
        {
                    // 創建禁用 DWA 規劃的請求
                    auto request = std::make_shared<campusrover_msgs::srv::PlannerFunction::Request>();
                    request->mode = campusrover_msgs::srv::PlannerFunction::Request::MODE_GLOBAL_PATH;
                    request->action.data = false;
                    request->obstacle_avoidance.data = true;
                    request->direction_inverse.data = false;
                    request->speed_parameter.linear.x = 0.4;
                    request->speed_parameter.angular.z = 0.4;
                    
                    // 發送請求
                    auto future = dwa_planner_client_->async_send_request(request);
                    
                    // 等待響應
                    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
                        rclcpp::FutureReturnCode::SUCCESS)
                    {
                        RCLCPP_INFO(this->get_logger(), "Stopped DWA Planner");
          }
          else
          {
                        RCLCPP_ERROR(this->get_logger(), "Failed to call DWA Planner Service");
            return;
          }
                    
          call_dwa_disable_srv_ = true;
          call_dwa_enable_srv_ = false;
        }
      }
      else
      {
        if(!call_dwa_disable_srv_){
          return;
        }
      }
    }
        
        // 如果 DWA 規劃已啟用，直接返回
    if(call_dwa_enable_srv_)
    {
      return;
    }
  }
  else
  {
        // 未啟用 DWA 避障，但有檢測到障礙物
    if(detect_front_obstacle)
    {
      if(front_ob_closest_dis <= obstacle_detect_min_dis_)
      {
        len_vel = 0.0;
      }
      else
      {
                len_vel = len_vel * (front_ob_closest_dis - obstacle_detect_min_dis_) / 
                          (obstacle_detect_max_dis_ - obstacle_detect_min_dis_);
            }
        }
    }

    // 如果接近路徑終點，減速
    if(closest_id >= static_cast<int>(following_path_.poses.size()) - 1)
    {
        len_vel = len_vel * 0.3;
    }

    // 如果啟用角速度依賴線速度調整
  if(enable_linear_depend_angular_)
  {
        if(std::abs(yaw_error) >= max_angle_of_linear_profile_)
    {
      len_vel = 0.0;
    }
        else if(std::abs(yaw_error) >= min_angle_of_linear_profile_ && 
                std::abs(yaw_error) < max_angle_of_linear_profile_)
    {
            len_vel = len_vel * (1.0 - ((std::abs(yaw_error) - min_angle_of_linear_profile_) / max_angle_of_linear_profile_));
    }
  }
  
    // 接近目標點時減速
  double min_reduce_dis = 0.8;
    double target_dist = std::sqrt(std::pow(following_path_.poses[following_path_.poses.size()-1].pose.position.x - robot_tf_pose_.position.x, 2) + 
                                std::pow(following_path_.poses[following_path_.poses.size()-1].pose.position.y - robot_tf_pose_.position.y, 2));
  if(target_dist < min_reduce_dis)
  {
        if(target_dist / min_reduce_dis > 1.0)
    {
      len_vel = max_linear_velocity_;
    }
        else if(target_dist / min_reduce_dis < 0.1)
    {
      len_vel = min_linear_velocity_;
    }
    else
    {
            len_vel = len_vel * (target_dist / min_reduce_dis);
    }
  }
    
    // 如果未啟用 DWA 避障，應用速度平滑
  if(!enble_dwa_obstacle_avoidance_)
  {
        twist_profile(len_vel, ang_vel);
    }
    
    // 發布速度命令
    twist_publish(len_vel, ang_vel);
}

/**
 * @brief 移動到目標方向
 */
void PathFollowing::moving_to_target_direction()
{
    // 計算方向誤差
    double yaw_error = target_yaw_ - robot_yaw_;
  angle_normalize(yaw_error);

    // 計算角速度
    double ang_vel = yaw_error * speed_pid_k_;
  
    // 限制最小角速度
  if(ang_vel > 0 && ang_vel < min_angular_velocity_)
  {
    ang_vel = min_angular_velocity_;
  }
  else if(ang_vel < 0 && ang_vel > -min_angular_velocity_)
  {
    ang_vel = -min_angular_velocity_;
  }

    // 發布速度命令
    twist_publish(0.0, ang_vel);
}

/**
 * @brief 發布速度命令
 * 
 * @param x 線速度（m/s）
 * @param z 角速度（rad/s）
 */
void PathFollowing::twist_publish(double x, double z)
{
    auto pub_twist = geometry_msgs::msg::Twist();

    // 限制角速度
  if(z > max_angular_velocity_)
  {
        pub_twist.angular.z = max_angular_velocity_;
  }
  else if(z < -max_angular_velocity_)
  {
    pub_twist.angular.z = -max_angular_velocity_;
  }
  else
  {
    pub_twist.angular.z = z;
  }
  
    // 設置線速度
  pub_twist.linear.x = x;
  
    // 發布速度命令
    twist_pub_->publish(pub_twist);
}

/**
 * @brief 速度平滑處理
 * 
 * @param profile_linear_x 目標線速度
 * @param profile_angular_z 目標角速度
 */
void PathFollowing::twist_profile(double &profile_linear_x, double &profile_angular_z)
{
  static double step_linear_x_ = 0.01;
  static double step_angular_z_ = 0.02;
  static double vel_tolerance = 0.015;

    // 平滑線速度
    if(std::abs(profile_linear_x - twist_linear_step_) <= vel_tolerance)
  {
    twist_linear_step_ = profile_linear_x;
  }
  else
  {
    if (profile_linear_x > twist_linear_step_)
    { 
      twist_linear_step_ = twist_linear_step_ + step_linear_x_;
    }
    else if (profile_linear_x < twist_linear_step_)
    {
            twist_linear_step_ = twist_linear_step_ - step_linear_x_ * 1.5;
    }
    else
    {
      twist_linear_step_ = profile_linear_x;
    }
  }

    // 平滑角速度
    if(std::abs(profile_angular_z - twist_angular_step_) <= vel_tolerance)
  {
        twist_angular_step_ = profile_angular_z;
  }
  else
  {
    if (profile_angular_z > twist_angular_step_)
    {
      twist_angular_step_ = twist_angular_step_ + step_angular_z_;
    }
    else if (profile_angular_z < twist_angular_step_)
    {
      twist_angular_step_ = twist_angular_step_ - step_angular_z_;
    }
    else
    {
      twist_angular_step_ = profile_angular_z;
    }
  }

    // 更新輸出值
  profile_linear_x = twist_linear_step_;
  profile_angular_z = twist_angular_step_;
}

/**
 * @brief 避讓功能
 */
// void PathFollowing::pullover_function()
// {
//     // ... [實現基於您的需求]
// }

/**
 * @brief 角度標準化到 [-π, π]
 * 
 * @param angle 需要標準化的角度（弧度）
 */
void PathFollowing::angle_normalize(double &angle)
{
    if(angle > M_PI)
    {
        angle = -2*M_PI + angle;
    }
    else if(angle < -M_PI)
    {
        angle = 2*M_PI + angle;
    }
}

/**
 * @brief 更新輸入路徑
 * 
 * @param path 新的路徑
 */
void PathFollowing::update_input_path(const nav_msgs::msg::Path &path)
{
  double roll, pitch, yaw;

    // 清空當前路徑
  get_following_path_ = false;
  following_path_.poses.clear();

    // 檢查路徑是否為空
  if(path.poses.size() == 0)
    return;
  
    // 使用正確的方法進行時間比較，將 builtin_interfaces::msg::Time 轉換為 rclcpp::Time
    auto path_stamp = path.header.stamp;
    rclcpp::Time path_time(path_stamp.sec, path_stamp.nanosec);
    
    // 直接比較兩個 rclcpp::Time 對象
    if (path_time < planner_start_time_)
    {
    return;
    }

    // 更新跟隨路徑
  following_path_.header.frame_id = path.header.frame_id;
  following_path_ = path;

    // 設置目標位姿
  target_pose_.header.frame_id = path.header.frame_id;
  target_pose_.pose.position = path.poses[path.poses.size()-1].pose.position;
  target_pose_.pose.orientation = path.poses[path.poses.size()-1].pose.orientation;

    // 從四元數獲取歐拉角
    tf2::Quaternion q(
        target_pose_.pose.orientation.x,
                    target_pose_.pose.orientation.y,
                    target_pose_.pose.orientation.z,
                    target_pose_.pose.orientation.w);
    tf2::Matrix3x3 m(q);

  m.getRPY(roll, pitch, yaw);
  target_yaw_ = yaw;

  get_following_path_ = true;
}

/**
 * @brief 調用電梯狀態檢查服務
 * 
 * @param request 服務請求
 */
void PathFollowing::elevator_status_check_call_service(
    const std::shared_ptr<campusrover_msgs::srv::ElevatorStatusChecker::Request> request)
{
    // 等待服務可用
    if (!elevator_status_check_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "Elevator status check service not available.");
    return;
  }

    RCLCPP_INFO(this->get_logger(), "Sending elevator status check request");
    
    // 發送請求
    auto future = elevator_status_check_client_->async_send_request(request);
    
    // 等待響應
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Elevator status check request processed successfully");
        }
        else
        {
        RCLCPP_ERROR(this->get_logger(), "Failed to call elevator status check service");
    }
}

// 添加 click_callback 實現
void PathFollowing::click_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // 根據您的業務邏輯處理按鈕事件
    RCLCPP_INFO(this->get_logger(), "Received joy message");
}
