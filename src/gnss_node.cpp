// gnss_node.cpp
#include "gnss/gnss_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace gnss {

GnssNode::GnssNode(const rclcpp::NodeOptions& options)
: rclcpp_lifecycle::LifecycleNode("gnss_node", options){
    RCLCPP_INFO(this->get_logger(), "GNSS Node is in Unconfigured state now.");
}

// on_configure: Called when the node transitions to the "configured" state
GnssNode::CallbackReturn GnssNode::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(),"Configuring GnssNode . . . ");

    //INITIALIZE PUBLISHERS
    switch_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("switch_odom", 10);

    //INITIALIZE BROADCASTERS
    odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    //INITIALIZE SUBSCRICERS
    fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/ublox_gps_node/fix", 10, std::bind(&GnssNode::fixCallback, this, _1));
    
    gnss_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/gnss_pose", 10, std::bind(&GnssNode::gnssPoseCallback, this, _1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odrive_odom", 10, std::bind(&GnssNode::odometryCallback, this, _1));

    RCLCPP_INFO(this->get_logger(),"Configuring GnssNOde COMPLETE!!! ");

    return CallbackReturn::SUCCESS;
}

// on_activate: Called when the node transitions to the "activated" state
GnssNode::CallbackReturn GnssNode::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(),"Activating GnssNode . . .");
    //Activate Publisher
    switch_odom_pub_->on_activate();

    //INITIALIZE Timer
    timer_ = this->create_wall_timer(50ms, std::bind(&GnssNode::timerCallback, this));

    //Activate Static BroadCaster
    sendStaticTransform();
    
    RCLCPP_INFO(this->get_logger(),"Activating GnssNode COMPLETE !!!");
    
    return CallbackReturn::SUCCESS;
}

// on_deactivate: Called when node transitions to the "deactivated/inactivated" state
GnssNode::CallbackReturn GnssNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(),"DEactivating GnssNode . . .");
    //Deactivate Publisher
    switch_odom_pub_->on_deactivate();

    //STOP,DISCARD Timer
    timer_.reset();
    
    RCLCPP_INFO(this->get_logger(),"DEctivating GnssNode COMPLETE !!!");
    
    return CallbackReturn::SUCCESS;
}

// on_cleanup: Called when the node transitions to Unconfigured state.
GnssNode::CallbackReturn GnssNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(),"Clearning up GnssNode . . .");

    //DISCAEDING
    timer_.reset();
    switch_odom_pub_.reset();
    odom_sub_.reset();
    gnss_pose_sub_.reset();
    fix_sub_.reset();
    static_broadcaster_.reset();
    odom_broadcaster_.reset();
    
    RCLCPP_INFO(this->get_logger(),"Cleaning Up GnssNode COMPLETE !!!");

    return CallbackReturn::SUCCESS;
}

// on_shutdown: Called when discarding the node. 
GnssNode::CallbackReturn GnssNode::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "Shutting down GnssNode...");
    // ここでは特に何もしませんが、将来的なリソース解放処理のために残しておきます (?)
    return CallbackReturn::SUCCESS;
}


void GnssNode::fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    gnss_component_.setFixStatus(msg->status.status);
    if (!ublox_received_) ublox_received_ = true;
}

void GnssNode::gnssPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    gnss_component_.setGnssPose(msg->pose.position.x, msg->pose.position.y);
    if (!gnss_pose_received_) gnss_pose_received_ = true;
}

void GnssNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    OdomInput data;
    data.x = msg->pose.pose.position.x;
    data.y = msg->pose.pose.position.y;
    data.yaw = yaw;
    data.vx = msg->twist.twist.linear.x;
    data.vth = msg->twist.twist.angular.z;
    gnss_component_.updateOdometry(data);

    if (!odometry_received_) odometry_received_ = true;
}

void GnssNode::timerCallback() {
    // 必要な全てのデータが最低1回は受信されるまで待機
    if (!ublox_received_ || !gnss_pose_received_ || !odometry_received_) {
        return;
    }

    // 計算コンポーネントに処理を依頼
    FusedOdom result = gnss_component_.computeFusedOdometry();

    // 結果をROSメッセージに変換
    auto current_time = this->get_clock()->now();

    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, result.yaw);
    geometry_msgs::msg::Quaternion odom_quat_msg = tf2::toMsg(odom_quat);

    // Publish Odometry message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = result.x;
    odom_msg.pose.pose.position.y = result.y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat_msg;
    odom_msg.twist.twist.linear.x = result.vx;
    odom_msg.twist.twist.angular.z = result.vth;
    switch_odom_pub_->publish(odom_msg);

    // Send TF transform
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = result.x;
    odom_trans.transform.translation.y = result.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat_msg;
    odom_broadcaster_->sendTransform(odom_trans);
}

void GnssNode::sendStaticTransform() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "odom";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.w = 1.0;
    static_broadcaster_->sendTransform(t);
}

} // namespace gnss