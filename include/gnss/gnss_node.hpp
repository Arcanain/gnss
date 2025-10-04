#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "gnss/gnss_component.hpp" // 作成したコンポーネントをインクルード

namespace gnss {

class GnssNode : public rclcpp::Node {
public:
    explicit GnssNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void gnssPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    void sendStaticTransform();

    // 計算ロジック本体のインスタンス生成
    GnssComponent gnss_component_;

    // ROS関連のメンバー
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gnss_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr switch_odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 最初のデータが届いたかを確認するフラグ
    bool ublox_received_ = false;
    bool gnss_pose_received_ = false;
    bool odometry_received_ = false;
};

} // namespace gnss