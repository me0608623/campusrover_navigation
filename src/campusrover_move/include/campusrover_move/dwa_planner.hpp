// dwa_planner.hpp
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/empty.hpp"
// #include "costmap_converter/msg/obstacle_array_msg.hpp"
// #include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>

#include "campusrover_msgs/srv/planner_function.hpp"
#include "campusrover_msgs/srv/elevator_status_checker.hpp"
#include "campusrover_move/srv/pull_over_path_generator.hpp"
#include "std_srvs/srv/empty.hpp"
#include <Eigen/Dense>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/transform_datatypes.h>
#include <costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <campusrover_msgs/srv/planner_function.hpp>
#include <campusrover_msgs/srv/elevator_status_checker.hpp>
#include <chrono>
#include <iostream>
#include <vector>
#include <memory>

class DWAPlanner : public rclcpp::Node
{
public:
  using Ptr = std::shared_ptr<DWAPlanner>;
  using SharedPtr = std::shared_ptr<DWAPlanner>;
  using SharedFuture = std::shared_future<std::shared_ptr<campusrover_msgs::srv::ElevatorStatusChecker::Response>>;
  DWAPlanner();

private:
  // Parameter setup
  void get_parameters();

  // Callbacks
  void elevatorPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void clickCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void CB_customObstacle(const costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr msg);

  void TimerCallback();
  void msgs_timerCallback();

  void ServiceCallback(
    const std::shared_ptr<campusrover_msgs::srv::PlannerFunction::Request> request,
    const std::shared_ptr<campusrover_msgs::srv::PlannerFunction::Response> response);

  bool ElevatorStatusCheckCallService(std::string node_name, std::string status);

  // Member variables
  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr            elevator_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr            global_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr   costmap_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr          click_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        odom_sub_;
  rclcpp::Subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr custom_obst_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr         twist_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              twist_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr             global_status_check_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr             global_status_publisher_;

  // Service Clients
  rclcpp::Client<campusrover_msgs::srv::ElevatorStatusChecker>::SharedPtr elevator_status_check_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr                        global_status_check_client_;
  rclcpp::Client<campusrover_msgs::srv::PlannerFunction>::SharedPtr       dwa_planner_client_;
  rclcpp::Client<campusrover_move::srv::PullOverPathGenerator>::SharedPtr pullover_planner_client_;

  // Service Server
  rclcpp::Service<campusrover_msgs::srv::PlannerFunction>::SharedPtr planner_function_service_;

  // Timers
  rclcpp::TimerBase::SharedPtr                                  control_timer_;
  rclcpp::TimerBase::SharedPtr                                  status_timer_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer>                             tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>                  tf_listener_;

  // Internal state / variables
  std::vector<Eigen::Vector2d> acc_pairs_;   // 加速度組合
  std::vector<Eigen::Vector2d> twist_pairs_; // 速度組合

  // Data structures
  nav_msgs::msg::Path globle_path_;              // 全域路徑
  geometry_msgs::msg::PoseStamped target_pose_;  // 目標位姿
  nav_msgs::msg::OccupancyGrid costmap_data_;    // 代價地圖數據
  geometry_msgs::msg::Pose robot_tf_pose_;       // 機器人 TF 位姿
  geometry_msgs::msg::PoseArray obstacle_poses_; // 障礙物位姿
  std_msgs::msg::String global_status_checker_msg_; // 全域狀態檢查消息

  std::string robot_frame_;    // 機器人座標框架名稱
  int status_msg_;             // 狀態訊息
  int path_sub_mode_;          // 路徑訂閱模式

  double arriving_range_dis_;   // 到達目標點距離閾值
  double arriving_range_angle_; // 到達目標方向角度閾值

  bool action_flag_;            // 啟動旗標
  bool get_globle_path_;        // 是否取得全域路徑
  bool get_costmap_data_;       // 是否取得地圖
  bool obstacle_stop_cmd_;      // 是否因障礙物停止
  bool arriving_end_point_;     // 是否到達目標點
  bool arriving_end_direction_; // 是否到達目標方向
  bool enble_costmap_obstacle_; // 是否啟用 costmap 障礙物
  bool direction_inverse_;      // 是否反向
  bool get_velocity_data_;      // 是否取得速度
  bool get_obstacle_data_;      // 是否取得障礙物

  double current_v_; // 當前線速度
  double current_w_; // 當前角速度

  bool enable_linear_depend_angular_; // 線速度是否依賴角速度
  double max_angle_of_linear_profile_; // 線速度 profile 最大角度
  double min_angle_of_linear_profile_; // 線速度 profile 最小角度

  double threshold_occupied_;    // 障礙物判斷閾值
  double footprint_max_x_;       // footprint 最大 x
  double footprint_min_x_;       // footprint 最小 x
  double footprint_max_y_;       // footprint 最大 y
  double footprint_min_y_;       // footprint 最小 y
  double obstacle_max_dis_;      // 障礙物最大距離
  double obstacle_min_dis_;      // 障礙物最小距離

  double robot_yaw_;    // 機器人朝向
  double speed_pid_k_;  // 速度 PID 係數
  double target_yaw_;   // 目標朝向

  double max_linear_acceleration_;  // 最大線加速度
  double max_angular_acceleration_; // 最大角加速度
  double max_linear_velocity_;      // 最大線速度
  double min_linear_velocity_;      // 最小線速度
  double max_angular_velocity_;     // 最大角速度
  double min_angular_velocity_;     // 最小角速度
  double target_point_dis_;         // 目標點距離

  double active_angular_;           // 主動角速度

  int trajectory_num_;              // 軌跡取樣數
  int trajectory_point_num_;        // 每條軌跡點數
  double delta_t_;                  // 取樣時間間隔
  double statu_v_;                  // 狀態線速度
  double statu_w_;                  // 狀態角速度

  // 目標函數權重
  double obstable_dis_weight_;     // 障礙物距離權重
  double target_heading_weight_;   // 目標朝向權重
  double velocity_weight_;         // 速度權重

  // Utility functions
  void InitialAccTrajectorySampling();
  void UpdateCampusRoverPoseFromTF();
  void check_arrive_point();
  void check_arrive_direction();
  void TwistPublish(double x, double z);
  bool moving_to_target_point();
  void moving_to_target_direction();
  void angle_normalize(double &angle);
  void VisualizePath(const std::vector<Eigen::Vector2d>& twist_pairs, double dt, int best_trajectory_id);
  geometry_msgs::msg::Pose predictPosition(double v_x, double v_w, double d_t);
  double calculateClosestObstacleDistance(double x, double y);

  // Constants
  static constexpr double PI = 3.14159265358979323846;
};
