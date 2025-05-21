// ROS2 基本功能
// #include "rclcpp/rclcpp.hpp"
#include "campusrover_move/dwa_planner.hpp"
#include <std_srvs/srv/empty.hpp>
// 標準 C++ 函式庫
#include <cmath>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Eigen 線性代數
// #include <Eigen/Dense>

// geometry_msgs 訊息
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

// // nav_msgs 訊息
// #include "nav_msgs/msg/path.hpp"
// #include "nav_msgs/msg/occupancy_grid.hpp"
// #include "nav_msgs/msg/odometry.hpp"

// tf2 套件（TF1 不再使用）
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // TF2 與 geometry 轉換
#include "tf2/utils.h"  // 若有用到 quaternion 工具

// sensor_msgs
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// 可視化
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// 自訂障礙物訊息（如果有升級）
#include <costmap_converter_msgs/msg/obstacle_msg.hpp>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>

// 自訂服務（需要升級為 ROS2 的 srv）
// #include "campusrover_msgs/srv/planner_function.hpp"
// #include "campusrover_msgs/srv/elevator_status_checker.hpp"

// 常數與命名空間
// #define M_PI 3.14159265358979323846
// using namespace std;

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

DWAPlanner::DWAPlanner()
: Node("dwa_planner")
{
  // 1. 參數讀取（若有自訂 get_parameters 函式）
  get_parameters();

  // Initialize TF buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 2. 訂閱者 Subscriber
  elevator_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "elevator_path", 10,
    std::bind(&DWAPlanner::elevatorPathCallback, this, std::placeholders::_1));

  global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "global_path", 10,
    std::bind(&DWAPlanner::globalPathCallback, this, std::placeholders::_1));

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "costmap", 10,
    std::bind(&DWAPlanner::costmapCallback, this, std::placeholders::_1));

  click_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "click_topic", 10,
    std::bind(&DWAPlanner::clickCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    std::bind(&DWAPlanner::odomCallback, this, std::placeholders::_1));

  custom_obst_sub_ = this->create_subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>(
    "obstacles", 10,
    std::bind(&DWAPlanner::CB_customObstacle, this, std::placeholders::_1));

  // 3. 發布者 Publisher
  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 50);
  twist_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("twist_path", 20);
  path_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("local_trajectories", 10);
  global_status_check_pub_ = this->create_publisher<std_msgs::msg::Empty>("reach_goal", 20);

  // 4. 服務客戶端 Service Client
  elevator_status_check_client_ =
    this->create_client<campusrover_msgs::srv::ElevatorStatusChecker>("elevator_status_checker");
  global_status_check_client_ =
    this->create_client<std_srvs::srv::Empty>("global_status_check");
//   dwa_planner_client_ =
//     this->create_client<campusrover_msgs::srv::PlannerFunction>("planner_function_dwa");
  pullover_planner_client_ =
    this->create_client<campusrover_move::srv::PullOverPathGenerator>("generate_pullover_path");

  // Setup service
  planner_function_service_ = this->create_service<campusrover_msgs::srv::PlannerFunction>(
    "planner_function_dwa", 
    std::bind(&DWAPlanner::ServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void DWAPlanner::get_parameters()
{
    // 從參數伺服器取得機器人座標框架名稱，預設 base_link
    robot_frame_ = this->declare_parameter<std::string>("robot_frame", "base_link");
    
    // 取得到達目標點的距離閾值，預設 0.2 公尺
    arriving_range_dis_ = this->declare_parameter<double>("arriving_range_dis", 0.2);
    
    // 取得到達目標方向的角度閾值，預設 0.1 弧度
    arriving_range_angle_ = this->declare_parameter<double>("arriving_range_angle", 0.1);

    // 最大線加速度，預設 3.0
    max_linear_acceleration_ = this->declare_parameter<double>("max_linear_acceleration", 3.0);
    
    // 最大角加速度，預設 8.0
    max_angular_acceleration_ = this->declare_parameter<double>("max_angular_acceleration", 8.0);
    
    // 最大線速度，預設 1.0
    max_linear_velocity_ = this->declare_parameter<double>("max_linear_velocity", 1.0);
    
    // 最小線速度，預設 0.0
    min_linear_velocity_ = this->declare_parameter<double>("min_linear_velocity", 0.0);
    
    // 最大角速度，預設 1.0
    max_angular_velocity_ = this->declare_parameter<double>("max_angular_velocity", 1.0);
    
    // 最小角速度，預設 1.0
    min_angular_velocity_ = this->declare_parameter<double>("min_angular_velocity", 1.0);
    
    // 跟隨路徑時，目標點與機器人距離閾值，預設 1.0
    target_point_dis_ = this->declare_parameter<double>("target_point_dis", 1.0);

    // costmap 障礙物判斷閾值，預設 10
    threshold_occupied_ = this->declare_parameter<double>("threshold_occupied", 10);
    
    // 機器人 footprint 最大/最小 x/y 範圍
    footprint_max_x_ = this->declare_parameter<double>("footprint_max_x", 1.5);
    footprint_min_x_ = this->declare_parameter<double>("footprint_min_x", -0.5);
    footprint_max_y_ = this->declare_parameter<double>("footprint_max_y", 0.5);
    footprint_min_y_ = this->declare_parameter<double>("footprint_min_y", -0.5);
    
    // 速度 PID 係數，預設 0.06
    speed_pid_k_ = this->declare_parameter<double>("speed_pid_k", 0.06);

    // 線速度與角度 profile 的最小/最大角度
    min_angle_of_linear_profile_ = this->declare_parameter<double>("min_angle_of_linear_profile", 0.1);
    max_angle_of_linear_profile_ = this->declare_parameter<double>("max_angle_of_linear_profile", 0.5);

    // 是否啟用 costmap 障礙物、反向、線速度依賴角速度
    enble_costmap_obstacle_ = this->declare_parameter<bool>("enble_costmap_obstacle", false);
    direction_inverse_ = this->declare_parameter<bool>("direction_inverse", false);
    enable_linear_depend_angular_ = this->declare_parameter<bool>("enable_linear_depend_angular", false);

    // DWA 軌跡取樣數量、每條軌跡點數、取樣時間間隔
    trajectory_num_ = this->declare_parameter<int>("trajectory_num", 44);
    trajectory_point_num_ = this->declare_parameter<int>("trajectory_point_num", 10);
    delta_t_ = this->declare_parameter<double>("delta_t", 0.3);
    
    // 目標函數權重
    obstable_dis_weight_ = this->declare_parameter<double>("obstable_dis_weight", 1.2);
    target_heading_weight_ = this->declare_parameter<double>("target_heading_weight", 1); 
    velocity_weight_ = this->declare_parameter<double>("velocity_weight", 1);
    
    // 障礙物最大/最小距離
    obstacle_max_dis_ = this->declare_parameter<double>("obstacle_max_dis", 2.0);
    obstacle_min_dis_ = this->declare_parameter<double>("obstacle_min_dis", 0.5);
    
    // 狀態速度
    statu_v_ = this->declare_parameter<double>("statu_v", 0.0);
    statu_w_ = this->declare_parameter<double>("statu_w", 0.0);

    // 初始化加速度取樣表
    InitialAccTrajectorySampling();
}

//-----------------------------------------------------------------------------------------------
// 初始化加速度取樣表，產生所有可能的線加速度與角加速度組合
void DWAPlanner::InitialAccTrajectorySampling()
{
    // 清空現有的加速度對
    acc_pairs_.clear();
    
    // 依據設定的取樣數量，雙層迴圈產生所有加速度組合
    for (int i = 0; i < trajectory_num_; i++)  // 外層迴圈，線加速度取樣
    {
        for (int j = 0; j < trajectory_num_ * 2.0; j++)  // 內層迴圈，角加速度取樣
        {
            // 計算線加速度與角加速度的取樣值
            Eigen::Vector2d acc_pair(
                -max_linear_acceleration_ + i * (2.0 * max_linear_acceleration_ / (trajectory_num_ - 1)),
                -max_angular_acceleration_ + j * (2.0 * max_angular_acceleration_ / ((trajectory_num_ * 2.0) - 1))
            );
            // 將取樣結果存入 acc_pairs_ 向量 
            acc_pairs_.push_back(acc_pair);
        }
    }
}

// 透過 TF 取得機器人在全域座標的即時姿態
void DWAPlanner::UpdateCampusRoverPoseFromTF()
{
    try {
        geometry_msgs::msg::TransformStamped transform;
        transform = tf_buffer_->lookupTransform(
            costmap_data_.header.frame_id, robot_frame_,
            tf2::TimePointZero, 
            tf2::durationFromSec(2.0));  // 最長等待時間為 2 秒

        // 將查詢到的座標與姿態存入 robot_tf_pose_
        robot_tf_pose_.position.x = transform.transform.translation.x;
        robot_tf_pose_.position.y = transform.transform.translation.y;
        robot_tf_pose_.position.z = transform.transform.translation.z;
        robot_tf_pose_.orientation.x = transform.transform.rotation.x;
        robot_tf_pose_.orientation.y = transform.transform.rotation.y;
        robot_tf_pose_.orientation.z = transform.transform.rotation.z;
        robot_tf_pose_.orientation.w = transform.transform.rotation.w;

        // 將四元數轉換為 roll, pitch, yaw
        tf2::Quaternion q(
            robot_tf_pose_.orientation.x,
            robot_tf_pose_.orientation.y,
            robot_tf_pose_.orientation.z,
            robot_tf_pose_.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        if (!direction_inverse_) {
            // 正向移動，直接取 yaw
            robot_yaw_ = yaw;
        } else {
            // 反向移動，yaw 加 pi
            robot_yaw_ = yaw + M_PI;
            angle_normalize(robot_yaw_);
        }
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "UpdateCampusRoverPoseFromTF: %s", ex.what());
    }
}
//-----------------------------------------------------------------------------------------------
void DWAPlanner::check_arrive_point()
{
  double dist = 0.0;
  dist = sqrt(pow(target_pose_.pose.position.x - robot_tf_pose_.position.x, 2) +
              pow(target_pose_.pose.position.y - robot_tf_pose_.position.y, 2));

  if (dist < arriving_range_dis_) {
    arriving_end_point_ = true;  // 設定已到達目標點
  } else {
    arriving_end_point_ = false;  // 設定尚未到達目標點
  }
}
//-----------------------------------------------------------------------------------------------
void DWAPlanner::check_arrive_direction()
{
  // 計算角度差異
  double angle_error = target_yaw_ - robot_yaw_;
  angle_normalize(angle_error);  // 正規化到 [-π, π]

  // 判斷是否達到目標方向
  if (abs(angle_error) < arriving_range_angle_) {
    arriving_end_direction_ = true;
  } else {
    arriving_end_direction_ = false;
    return;
  }

  // 發送狀態訊息
  if (get_globle_path_) {
    if (path_sub_mode_ == campusrover_msgs::srv::PlannerFunction::Request::MODE_ELEVATOR_PATH) {
      // 電梯導航模式，調用服務
      bool service_result = ElevatorStatusCheckCallService("dwa_planner", "arrived_target_direction");
      RCLCPP_INFO(this->get_logger(), "Elevator status check service called, result: %s", service_result ? "true" : "false");
    } else if (path_sub_mode_ == campusrover_msgs::srv::PlannerFunction::Request::MODE_GLOBAL_PATH) {
      // 一般全域導航模式，發布消息
      global_status_checker_msg_.data = "arrived_target_direction";
      global_status_publisher_->publish(global_status_checker_msg_);
    }
  }
}

//----------------------------------------------------------------------------------------------
void DWAPlanner::TimerCallback()  // 定時器 callback，負責主控制流程
{
    if (!action_flag_) {  // 如果未啟動則直接返回
        return;
    }
    
    if (path_sub_mode_ == campusrover_msgs::srv::PlannerFunction::Request::MODE_BUTTON_PARKING) {  // 如果是按鈕停車模式
        TwistPublish(max_linear_velocity_, active_angular_);  // 直接發布最大速度與主動角速度
    } else {  // 其他模式
        if (get_globle_path_) {  // 如果已取得全域路徑
            if (get_velocity_data_) {  // 如果已取得速度資料
                if (get_costmap_data_ || get_obstacle_data_ || !enble_costmap_obstacle_) {  // 如果有地圖、障礙物或未啟用 costmap 障礙物
                    UpdateCampusRoverPoseFromTF();  // 更新機器人姿態

                    if (!arriving_end_direction_) {  // 如果尚未到達目標方向
                        if (!arriving_end_point_) {  // 如果尚未到達目標點
                            check_arrive_point();  // 檢查是否到達目標點
                            if (arriving_end_point_) {  // 如果到達目標點
                                moving_to_target_direction();  // 進入朝向目標方向模式
                                return;
                            }
                            moving_to_target_point();  // 否則繼續跟隨路徑
                        } else {  // 如果已到達目標點
                            check_arrive_direction();  // 檢查是否到達目標方向
                            if (arriving_end_direction_) {  // 如果到達目標方向
                                status_msg_ = 4;  // 狀態設為到達終點
                                TwistPublish(0.0, 0.0);  // 停止移動
                                return;
                            }
                            moving_to_target_direction();  // 否則繼續朝向目標方向
                        }
                    } else {  // 如果已到達目標方向
                        status_msg_ = 4;  // 狀態設為到達終點
                        TwistPublish(0.0, 0.0);  // 停止移動
                    }
                } else {  // 如果沒有地圖或障礙物資料
                    status_msg_ = 2;  // 狀態設為等待地圖
                    TwistPublish(0.0, 0.0);  // 停止移動
                }
            } else {  // 如果沒有速度資料
                status_msg_ = 5;  // 狀態設為等待 odom
                TwistPublish(0.0, 0.0);  // 停止移動
            }
        } else {  // 如果沒有全域路徑
            status_msg_ = 1;  // 狀態設為等待路徑
            TwistPublish(0.0, 0.0);  // 停止移動
        }
    }
}

//-----------------------------------------------------------------------------------------------
void DWAPlanner::msgs_timerCallback()  // 狀態訊息定時提示 callback
{
    if (status_msg_ == 1) {  // 沒有路徑
        RCLCPP_WARN(this->get_logger(), "dwa_planner : Without Globle Path to follow, Waiting for the Path input");
        status_msg_ = 0;
    } else if (status_msg_ == 2) {  // 沒有地圖
        RCLCPP_WARN(this->get_logger(), "dwa_planner : Without costmap input , Waiting for the costmap input");
        status_msg_ = 0;
    } else if (status_msg_ == 3) {  // 偵測到障礙物
        RCLCPP_INFO(this->get_logger(), "dwa_planner : detect obstacle");
        status_msg_ = 0;
    } else if (status_msg_ == 4) {  // 到達終點
        RCLCPP_INFO(this->get_logger(), "dwa_planner : Arrival the destination");
        status_msg_ = 0;
    } else if (status_msg_ == 5) {  // 沒有 odom
        RCLCPP_INFO(this->get_logger(), "dwa_planner : Without odom input , Waiting for the odom input");
        status_msg_ = 0;
    } else {  // 其他狀態
        status_msg_ = 0;
    }
}

//-----------------------------------------------------------------------------------------------
bool DWAPlanner::moving_to_target_point()
{
  bool best_id_founded = false;
  double best_value = 0.0;
  int best_id = 0;
  
  // 初始化評分向量
  std::vector<double> objective_function_values(twist_pairs_.size());
  
  // 遍歷所有軌跡
  for(int p=0; p < twist_pairs_.size(); p++)
  {
    // ... 現有代碼 ...
  }
  
  // 計算到終點的距離
  double target_dist = sqrt(pow(globle_path_.poses[globle_path_.poses.size()-1].pose.position.x - robot_tf_pose_.position.x, 2) +
                           pow(globle_path_.poses[globle_path_.poses.size()-1].pose.position.y - robot_tf_pose_.position.y, 2));
  
  if(best_id_founded){ // 如果找到了最佳軌跡
    // ... 現有代碼 ...
  }else{
    // ... 現有代碼 ...
  }
  
  VisualizePath(twist_pairs_, delta_t_*12.0, best_id); // 在RViz中可視化所有軌跡，突出顯示最佳軌跡
  
  return true;
}

//-----------------------------------------------------------------------------------------------
void DWAPlanner::moving_to_target_direction() // 朝向目標方向旋轉的函數
{
    static double yaw_error; // 偏航角誤差
    static double ang_vel;   // 角速度

    yaw_error = target_yaw_ - robot_yaw_;    // 計算目標偏航角與當前偏航角的差值
    angle_normalize(yaw_error);              // 將角度誤差正規化到[-π, π]範圍

    ang_vel = yaw_error * speed_pid_k_;      // 使用比例控制計算角速度

    if (ang_vel > 0.0 && ang_vel < min_angular_velocity_)       // 如果正向角速度太小
    {
        ang_vel = min_angular_velocity_;      // 設為最小角速度
    }
    else if (ang_vel < 0.0 && ang_vel > -min_angular_velocity_)  // 如果反向角速度太小
    {
        ang_vel = -min_angular_velocity_;     // 設為最小負角速度
    }

    TwistPublish(0.0, ang_vel);              // 發布角速度指令，線速度為0
}

//-----------------------------------------------------------------------------------------------
void DWAPlanner::TwistPublish(double x, double z) // 發布速度指令的函數
{
    static geometry_msgs::msg::Twist pub_twist; // 用於發布的速度消息

    // 限制角速度在允許範圍內
    if (z > max_angular_velocity_)                // 如果角速度超過最大值
    {
        pub_twist.angular.z = max_angular_velocity_;    // 限制為最大角速度
    }
    else if (z < -max_angular_velocity_)         // 如果角速度小於最小值
    {
        pub_twist.angular.z = -max_angular_velocity_;   // 限制為最小角速度
    }
    else                                         // 角速度在允許範圍內
    {
        pub_twist.angular.z = z;                // 直接使用輸入的角速度
    }

    // 根據角速度調整線速度
    if (std::abs(z) > max_angular_velocity_ * 0.7)    // 如果角速度大於最大值的70%
    {
        pub_twist.linear.x = x * (1.0 - ((std::abs(z) - (max_angular_velocity_ * 0.7))
                               / (2.0 * max_angular_velocity_))); // 降低線速度
    }
    else                                          // 角速度較小時
    {
        pub_twist.linear.x = x;                  // 直接使用輸入的線速度
    }

    twist_pub_->publish(pub_twist);               // 發布速度指令到話題
}

//-----------------------------------------------------------------------------------------------
void DWAPlanner::angle_normalize(double &angle) // 角度正規化函數，將角度限制在[-π, π]範圍內
{
    if (angle > M_PI)              // 如果角度大於π
    {
        angle = angle - 2.0 * M_PI; // 減去2π
    }
    else if (angle < -M_PI)        // 如果角度小於-π
    {
        angle = angle + 2.0 * M_PI; // 加上2π
    }
}

//-----------------------------------------------------------------------------------------------
// 電梯模式下的路徑接收回呼函式
void DWAPlanner::elevatorPathCallback(const nav_msgs::msg::Path::SharedPtr path)
{
    static geometry_msgs::msg::PoseStamped pose; // 用來暫存每個路徑點的 Pose
    static double roll, pitch, yaw;              // 用來儲存轉換後的歐拉角（roll, pitch, yaw）

    // 如果當前模式不是電梯路徑模式，就不處理這份路徑資料
    if (path_sub_mode_ != campusrover_msgs::srv::PlannerFunction::Request::MODE_ELEVATOR_PATH)
    {
        return; // 結束函式
    }

    // 如果收到的 path 是空的（沒有任何路徑點），就不處理
    if (path->poses.empty())
    {
        return;
    }

    // 將接收到的 path 的座標框架（例如 "map"）儲存到全域路徑的 header
    globle_path_.header.frame_id = path->header.frame_id;

    // 清空之前的路徑資料，準備儲存新的 path
    globle_path_.poses.clear();

    // 遍歷接收到的 path 裡所有的路徑點
    for (size_t i = 0; i < path->poses.size(); ++i)
    {
        // 將每一個點的位置資訊複製到暫存的 pose
        pose.pose.position = path->poses[i].pose.position;
        // 將每一個點的姿態（四元數）資訊也複製進來
        pose.pose.orientation = path->poses[i].pose.orientation;

        // 把這個點加入到全域路徑陣列中
        globle_path_.poses.push_back(pose);

        // 如果是最後一個點（目標點）
        if (i == path->poses.size() - 1)
        {
            // 設定最終目標點（target_pose_）為這個點
            target_pose_.header.frame_id = path->header.frame_id;
            target_pose_.pose.position    = path->poses[i].pose.position;
            target_pose_.pose.orientation = path->poses[i].pose.orientation;
        }
    }

    // 將目標點的四元數轉換為歐拉角，以取得目標的 yaw（朝向角度）
    tf2::Quaternion q(
        target_pose_.pose.orientation.x,
        target_pose_.pose.orientation.y,
        target_pose_.pose.orientation.z,
        target_pose_.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw); // 將四元數轉為 roll, pitch, yaw

    // 將目標方向儲存到全域變數 target_yaw_
    target_yaw_ = yaw;

    // 設定路徑已成功接收的旗標，讓主控制流程可以開始進行導航
    get_globle_path_ = true;
}

//-----------------------------------------------------------------------------------------------
// 全域路徑的回呼函式：接收 /global_path 話題後更新目標路徑與目標姿態
void DWAPlanner::globalPathCallback(const nav_msgs::msg::Path::SharedPtr path)
{
    static geometry_msgs::msg::PoseStamped pose; // 暫存每個路徑點的 Pose
    static double roll, pitch, yaw;              // 用於將四元數轉換為歐拉角（roll, pitch, yaw）

    // if(path_sub_mode_ != campusrover_msgs::PlannerFunction::Request::MODE_GLOBAL_PATH)
    // {
    //   return; // 如果不是全域模式，直接跳出
    // }
    // 若接收到的 path 中沒有任何點，直接忽略
    if (path->poses.empty()) {
        return;
    }

    // 設定全域路徑的參考座標系（例如 "map" 或 "odom"）
    globle_path_.header.frame_id = path->header.frame_id;

    // 清除先前儲存的全域路徑點，準備接收新的
    globle_path_.poses.clear();

    // 遍歷接收到的所有 Pose 資料
    for (size_t i = 0; i < path->poses.size(); ++i) {
        // 將位置資訊從輸入 path 複製到暫存 pose
        pose.pose.position    = path->poses[i].pose.position;
        // 將姿態（四元數）資訊也複製進來
        pose.pose.orientation = path->poses[i].pose.orientation;

        // 加入到全域路徑向量中
        globle_path_.poses.push_back(pose);

        // 若是最後一個點，設為目標位置
        if (i == path->poses.size() - 1) {
            target_pose_.header.frame_id    = path->header.frame_id;              // 設定目標點參考座標系
            target_pose_.pose.position      = path->poses[i].pose.position;       // 設定目標位置
            target_pose_.pose.orientation   = path->poses[i].pose.orientation;    // 設定目標朝向（四元數）
        }
    }

    // 從四元數轉換為歐拉角（取得目標朝向的 yaw）
    tf2::Quaternion q(
        target_pose_.pose.orientation.x,
        target_pose_.pose.orientation.y,
        target_pose_.pose.orientation.z,
        target_pose_.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw); // 取得 roll, pitch, yaw（這裡只需要 yaw）

    // 將 yaw 存為目標朝向角度，供導航使用
    target_yaw_ = yaw;

    // 若是第一次接收到全域路徑，輸出提示訊息
    if (!get_globle_path_) {
        RCLCPP_INFO(this->get_logger(), "dwa_planner : Get the globle path input!!");
    }

    // 設定路徑已接收完成，主控制流程可以開始導航
    get_globle_path_ = true;
}
void DWAPlanner::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
    geometry_msgs::msg::Pose ob_pose;               // 存放障礙物在 base frame 中的簡單座標
    geometry_msgs::msg::PoseStamped ob_posestamped; // 存放障礙物原始座標（包含 frame_id）
    geometry_msgs::msg::PoseStamped base_ob_pose;   // 轉換成機器人 frame 的障礙物 pose

    // TF2 轉換用的 buffer 與 listener（傳入 node clock）
    static tf2_ros::Buffer tfBuffer(this->get_clock());
    static tf2_ros::TransformListener tfListener(tfBuffer);
    static geometry_msgs::msg::TransformStamped transformStamped; // 暫存轉換結果

    double data;   // costmap 中每格的值（-1:unknown, 0:free, 100:occupied）
    double value;  // 對應 index 值

    // 如果未啟用 costmap 障礙物功能，直接跳出
    if (!enble_costmap_obstacle_) {
        return;
    }

    // 儲存解析度（每格幾公尺），供後續障礙物位置換算使用
    costmap_data_.info.resolution = map->info.resolution;

    // 清除之前儲存的障礙物座標
    obstacle_poses_.poses.clear();

    // ============================== CASE 1：地圖與機器人座標系相同 ==============================
    if (map->header.frame_id == robot_frame_)
    {
        for (size_t i = 0; i < map->data.size(); ++i) // 遍歷每一格
        {
            data = map->data[i]; // 取得格子的值（0~100）
            value = static_cast<double>(i); // 索引值

            if (std::abs(data) > threshold_occupied_) // 超過占據閾值，視為障礙物
            {
                // 將 1D 索引轉換為 2D 格子位置 → 再換算為世界座標
                ob_pose.position.y = (std::floor(value / map->info.width) * map->info.resolution)
                                     + map->info.origin.position.y;
                ob_pose.position.x = ((value - map->info.width * std::floor(value / map->info.width))
                                     * map->info.resolution) + map->info.origin.position.x;

                // 判斷是否進入機器人 footprint 區域
                if (ob_pose.position.x < footprint_max_x_ && ob_pose.position.x > footprint_min_x_ &&
                    ob_pose.position.y < footprint_max_y_ && ob_pose.position.y > footprint_min_y_)
                {
                    obstacle_stop_cmd_ = true;  // 設定旗標要求停止
                    status_msg_ = 3;            // 訊息代碼 3：偵測到障礙物
                    break;                      // 一旦進入 footprint 就停止處理
                }
                else
                {
                    obstacle_stop_cmd_ = false; // 沒進入 footprint，清除停止指令
                }

                // 將障礙物儲存進列表，用於後續軌跡評估
                obstacle_poses_.poses.push_back(ob_pose);
            }
        }
    }
    // ============================== CASE 2：地圖與機器人座標系不同（需 TF 轉換） ==============================
    else
    {
        ob_posestamped.header.frame_id = map->header.frame_id; // 設定原始 frame

        for (size_t i = 0; i < map->data.size(); ++i) // 遍歷每一格
        {
            data = map->data[i];

            if (std::abs(data) > threshold_occupied_) // 超過閾值，視為障礙物
            {
                value = static_cast<double>(i);

                // 計算原始世界座標（還不是 base frame）
                ob_posestamped.pose.position.y = (std::floor(value / map->info.width) * map->info.resolution)
                                                 + map->info.origin.position.y;
                ob_posestamped.pose.position.x = ((value - map->info.width * std::floor(value / map->info.width))
                                                 * map->info.resolution) + map->info.origin.position.x;

                // 嘗試將該點從地圖 frame 轉換到機器人 base frame
                try
                {
                    // 將障礙物位置轉換到機器人坐標系
                    tfBuffer.transform(ob_posestamped, base_ob_pose, robot_frame_, 
                        tf2::durationFromSec(2.0));
                }
                catch (tf2::TransformException &ex)
                {
                    RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
                    return;
                }

                // 更新轉換後的障礙物座標
                ob_pose.position.y = base_ob_pose.pose.position.y;
                ob_pose.position.x = base_ob_pose.pose.position.x;

                // 檢查是否進入 footprint
                if (ob_pose.position.x < footprint_max_x_ && ob_pose.position.x > footprint_min_x_ &&
                    ob_pose.position.y < footprint_max_y_ && ob_pose.position.y > footprint_min_y_)
                {
                    obstacle_stop_cmd_ = true;  // 要求停止
                    status_msg_ = 3;            // 更新狀態碼
                    break;
                }
                else
                {
                    obstacle_stop_cmd_ = false;
                }

                // 儲存障礙物座標
                obstacle_poses_.poses.push_back(ob_pose);
            }
        }
    }

    // 若第一次接收到 costmap，顯示提示訊息
    if (!get_costmap_data_) {
        RCLCPP_INFO(this->get_logger(), "dwa_planner : Get the Costmap input!!");
    }

    // 設定 costmap 資料已接收
    get_costmap_data_ = true;
}
//-----------------------------------------------------------------------------------------------
// 當訂閱到 odometry 資料時會呼叫此函式，用來取得機器人的實際速度
void DWAPlanner::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // 取得目前機器人的線速度（x 方向）
    current_v_ = msg->twist.twist.linear.x;

    // 取得目前機器人的角速度（z 軸旋轉）
    current_w_ = msg->twist.twist.angular.z;

    // 如果是第一次接收到 odom，印出提示資訊
    if (!get_velocity_data_) {
        RCLCPP_INFO(this->get_logger(), "dwa_planner : Get the Odom input!!");
    }

    // 設定旗標，表示 odom 資料已成功接收
    get_velocity_data_ = true;
}

//-----------------------------------------------------------------------------------------------
// 回呼函式：處理從 costmap_converter 發布的障礙物陣列訊息
void DWAPlanner::CB_customObstacle(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
{
    // 建立 TF2 轉換工具，用於不同座標系之間的轉換
    static tf2_ros::Buffer tfBuffer(this->get_clock());
    static tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::msg::Pose ob_pose;               // 用於儲存障礙物座標（無 frame）
    geometry_msgs::msg::PoseStamped obstacle_g;     // 原始 frame 中的 pose（需轉換）
    geometry_msgs::msg::PoseStamped obstacle_r;     // 機器人底座 frame 中的 pose（轉換後）

    // 清除上一輪的障礙物資料，準備儲存新的
    obstacle_poses_.poses.clear();

    // ============================== CASE 1：若障礙物與機器人在同一座標系 ==============================
    if (msg->header.frame_id == robot_frame_)
    {
        for (size_t i = 0; i < msg->obstacles.size(); ++i) // 遍歷每一個障礙物
        {
            // 取第一個頂點（通常代表障礙物中心或某邊界點）
            ob_pose.position.x = msg->obstacles[i].polygon.points[0].x;
            ob_pose.position.y = msg->obstacles[i].polygon.points[0].y;

            // 加入到障礙物清單中
            obstacle_poses_.poses.push_back(ob_pose);
        }
    }
    // ============================== CASE 2：障礙物與機器人不在同一座標系（需轉換） ==============================
    else
    {
        for (size_t i = 0; i < msg->obstacles.size(); ++i) // 遍歷每一個障礙物
        {
            // 設定障礙物原始座標系與座標點
            obstacle_g.header.frame_id = msg->obstacles[i].header.frame_id;
            obstacle_g.pose.position.x = msg->obstacles[i].polygon.points[0].x;
            obstacle_g.pose.position.y = msg->obstacles[i].polygon.points[0].y;

            try
            {
                // 嘗試將該障礙物座標轉換為 robot_frame 座標系下的位置
                tfBuffer.transform(
                    obstacle_g, obstacle_r, robot_frame_,
                    tf2::durationFromSec(2.0));
            }
            catch (const tf2::TransformException &ex)
            {
                // 若轉換失敗，印出警告並跳出
                RCLCPP_WARN(this->get_logger(), "ob : %s", ex.what());
                return;
            }

            // 將轉換後的障礙物位置儲存
            ob_pose.position.x = obstacle_r.pose.position.x;
            ob_pose.position.y = obstacle_r.pose.position.y;

            // 加入清單中
            obstacle_poses_.poses.push_back(ob_pose);
        }
    }

    // 成功接收到自訂障礙物資料，設定旗標為 true
    get_obstacle_data_ = true;
}

//-----------------------------------------------------------------------------------------------
// 根據給定的速度與時間，預測機器人未來的相對位置與朝向（回傳 Pose）
geometry_msgs::msg::Pose DWAPlanner::predictPosition(double v_x, double v_w, double d_t)
{
    static geometry_msgs::msg::Pose predict_pose;         // 儲存預測的位置與姿態
    static tf2::Quaternion pose_q_tf;                     // TF 格式的四元數（用來產生旋轉）
    static geometry_msgs::msg::Quaternion pose_q_msg;     // ROS2 標準格式的四元數
    double d_angle;                                       // delta angle = v_w * d_t（旋轉角）

    // ============================== CASE 1：只有線速度，無角速度（直線前進） ==============================
    if (v_w == 0.0)
    {
        predict_pose.position.x = v_x * d_t;  // 沿 x 軸向前移動
        predict_pose.position.y = 0.0;        // y 方向無變化
        predict_pose.position.z = 0.0;        // z 不考慮高度變化（2D）

        pose_q_tf.setRPY(0.0, 0.0, 0.0);      // 朝向不變
        pose_q_msg = tf2::toMsg(pose_q_tf);   // 轉成 ROS2 四元數格式

        predict_pose.orientation = pose_q_msg; // 設定姿態
    }
    // ============================== CASE 2：只有角速度，無線速度（原地旋轉） ==============================
    else if (v_x == 0.0)
    {
        d_angle = v_w * d_t;                  // 計算旋轉角度
        pose_q_tf.setRPY(0.0, 0.0, d_angle);   // 產生旋轉四元數（繞 Z 軸）
        pose_q_msg = tf2::toMsg(pose_q_tf);

        predict_pose.position.x = 0.0;         // 原地不動
        predict_pose.position.y = 0.0;
        predict_pose.position.z = 0.0;
        predict_pose.orientation = pose_q_msg; // 只設定角度
    }
    // ============================== CASE 3：同時有線與角速度（曲線前進） ==============================
    else
    {
        d_angle = v_w * d_t;                  // 計算旋轉角
        pose_q_tf.setRPY(0.0, 0.0, d_angle);   // 產生朝向
        pose_q_msg = tf2::toMsg(pose_q_tf);

        // 根據旋轉角度，計算曲線位移後的 x, y
        predict_pose.position.x = v_x * d_t * std::cos(d_angle);
        predict_pose.position.y = v_x * d_t * std::sin(d_angle);
        predict_pose.position.z = 0.0;
        predict_pose.orientation = pose_q_msg; // 設定預測姿態
    }

    return predict_pose; // 回傳預測結果（包含位置與朝向）
}

//-----------------------------------------------------------------------------------------------
// 計算座標點 (x, y) 到所有障礙物的最短距離（用於目標函數評估）
double DWAPlanner::calculateClosestObstacleDistance(double x, double y)
{
    double min_distance = obstacle_max_dis_;

    // 遍歷所有障礙物位置
    for (const auto &pose : obstacle_poses_.poses) {
        // 使用歐氏距離公式計算距離
        double dis = std::sqrt(
            std::pow(pose.position.x - x, 2.0) +
            std::pow(pose.position.y - y, 2.0)
        );

        if (dis < min_distance) {
            min_distance = dis;
        }
    }

    return min_distance;
}

//-----------------------------------------------------------------------------------------------
// 在 RViz 中畫出所有 DWA 模擬的速度軌跡（候選路徑）
// twist_pairs：每一條軌跡對應的 (v, w)
// dt：模擬時間長度
// best_trajectory_id：最佳軌跡的 index，用綠色標出
void DWAPlanner::VisualizePath(
    const std::vector<Eigen::Vector2d> &twist_pairs,
    double dt,
    int best_trajectory_id)
{
    visualization_msgs::msg::MarkerArray all_rollOuts;       // 儲存所有要畫的軌跡線條
    visualization_msgs::msg::Marker lane_waypoint_marker;    // 單一條軌跡用的 marker

    double s_x, s_y;   // 模擬的當前位置（相對座標）
    double theta;      // 模擬的當前角度（yaw）

    // 初始化每條軌跡的 marker 參數
    lane_waypoint_marker.header.frame_id = robot_frame_;            // 使用機器人 base frame
    lane_waypoint_marker.header.stamp = rclcpp::Time(0);            // 設定時間戳為 0（即時）
    lane_waypoint_marker.ns = "path_array_marker";                  // marker 命名空間
    lane_waypoint_marker.type = visualization_msgs::msg::Marker::LINE_STRIP; // 線段集合
    lane_waypoint_marker.action = visualization_msgs::msg::Marker::ADD;      // 新增 marker
    lane_waypoint_marker.scale.x = 0.01;  // 線條寬度
    lane_waypoint_marker.scale.y = 0.01;
    lane_waypoint_marker.frame_locked = false; // frame 鎖定與否

    for (int i = 0; i < static_cast<int>(twist_pairs.size()); ++i) // 對每一組速度取樣進行模擬
    {
        lane_waypoint_marker.points.clear(); // 清空點集合
        lane_waypoint_marker.id = i;         // 每一條軌跡需要唯一 ID

        // 計算每一步的距離和角度
        double step_dis   = twist_pairs[i][0] * dt / trajectory_point_num_;
        double step_theta = twist_pairs[i][1] * dt / trajectory_point_num_;

        s_x = s_y = 0.0;   // 起始位置為 (0, 0)
        theta = step_theta; // 初始角度

        // 模擬整條軌跡上的每個點（每一小段）
        for (int j = 0; j < trajectory_point_num_; ++j)
        {
            geometry_msgs::msg::Point point;

            s_x += step_dis * std::cos(theta); // 根據角度更新 x
            s_y += step_dis * std::sin(theta); // 根據角度更新 y

            point.x = s_x;
            point.y = s_y;

            lane_waypoint_marker.points.push_back(point); // 加入點到軌跡中

            theta += step_theta; // 持續轉彎
        }

        if (best_trajectory_id == i) // 若是最佳軌跡
        {
            lane_waypoint_marker.color.r = 0.0; // 綠色
            lane_waypoint_marker.color.g = 1.0;
            lane_waypoint_marker.color.b = 0.0;
            lane_waypoint_marker.color.a = 1.0; // 不透明
        }
        else
        {
            lane_waypoint_marker.color.r = 1.0; // 紫色（非最佳）
            lane_waypoint_marker.color.g = 0.0;
            lane_waypoint_marker.color.b = 1.0;
            lane_waypoint_marker.color.a = 0.3; // 半透明
        }

        all_rollOuts.markers.push_back(lane_waypoint_marker); // 加入到 marker array
    }

    path_marker_pub_->publish(all_rollOuts); // 發布所有軌跡到 RViz 顯示
}

//-----------------------------------------------------------------------------------------------
// 發送服務請求給電梯狀態檢查器，用來通知電梯系統：導航已完成（到達門口/方向朝向完成）
bool DWAPlanner::ElevatorStatusCheckCallService(std::string node_name, std::string status)
{
  // 在建立請求和調用服務之前檢查服務客戶端是否已初始化
  if (!elevator_status_check_client_) {
    elevator_status_check_client_ = this->create_client<campusrover_msgs::srv::ElevatorStatusChecker>("elevator_status_checker");
    RCLCPP_INFO(this->get_logger(), "Initialized elevator status checker client");
  }

  // 創建請求對象
  auto request = std::make_shared<campusrover_msgs::srv::ElevatorStatusChecker::Request>();
  
  // 為 String 類型的 node_name 參數賦值
  request->node_name.data = node_name;
  
  // 為 Bool 類型的 status 參數賦值
  request->status.data = (status == "arrived_target_direction");
  
  RCLCPP_INFO(this->get_logger(), "Sending elevator status check request: node_name=%s, status=%s",
              node_name.c_str(), status.c_str());

  // 等待服務可用
  if (!elevator_status_check_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(this->get_logger(), "Elevator status checker service not available");
    return false;
  }

  // 發送請求
  auto future = elevator_status_check_client_->async_send_request(request);
  
  // 等待響應
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != 
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to call elevator status checker service");
    return false;
  }
  
  // 服務調用成功
  RCLCPP_INFO(this->get_logger(), "Successfully called elevator status checker service");
  return true;
}

//-----------------------------------------------------------------------------------------------
// 服務回呼函式：處理來自外部的導航參數與啟動命令
void DWAPlanner::ServiceCallback(
    const std::shared_ptr<campusrover_msgs::srv::PlannerFunction::Request> request,
    const std::shared_ptr<campusrover_msgs::srv::PlannerFunction::Response> response)
{
    // 記錄上次的模式，用於檢測模式變更
    static int last_mode = path_sub_mode_;
    
    // 設置路徑模式和激活狀態
    path_sub_mode_ = request->mode;
    action_flag_ = request->action.data;
    
    // 輸出調試信息
    RCLCPP_INFO(this->get_logger(), "Service callback received:");
    RCLCPP_INFO(this->get_logger(), "  path_mode: %d", path_sub_mode_);
    RCLCPP_INFO(this->get_logger(), "  activation: %s", action_flag_ ? "true" : "false");
    
    // 如果不激活導航或者路徑模式變更，重置某些狀態
    if (!action_flag_ || (last_mode != path_sub_mode_)) {
        get_globle_path_ = false;
    }
    
    // 更新上次模式
    last_mode = path_sub_mode_;
    
    // 沒有 success 字段，無需設置
}

void DWAPlanner::clickCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // 這裡是處理搖桿訊息的邏輯
  // 可以根據實際需求添加更多處理
  RCLCPP_DEBUG(this->get_logger(), "Received joystick message");
  
  // 這裡只是一個簡單的空實現以滿足鏈接要求
}

int main(int argc, char ** argv)
{
  
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DWAPlanner>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}

