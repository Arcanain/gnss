#include "gnss/gnss_component.hpp"
#include <iostream> // RCLCPP_INFOの代わりに標準出力を使用

namespace gnss {

GnssComponent::GnssComponent() = default;

void GnssComponent::setFixStatus(int status) {
    fix_flag_ = (status == 2);
}

void GnssComponent::setGnssPose(double x, double y) {
    gnss_x_ = x;
    gnss_y_ = y;
}

void GnssComponent::updateOdometry(const OdomInput& odom_data) {
    odrive_yaw_ = odom_data.yaw;
    vx_ = odom_data.vx;
    vth_ = odom_data.vth;

    // 移動量を計算
    diff_odrive_x_ = odom_data.x - pre_odrive_x_;
    diff_odrive_y_ = odom_data.y - pre_odrive_y_;
    diff_odrive_yaw_ = odom_data.yaw - pre_odrive_yaw_;

    pre_odrive_x_ = odom_data.x;
    pre_odrive_y_ = odom_data.y;
    pre_odrive_yaw_ = odom_data.yaw;
}

FusedOdom GnssComponent::computeFusedOdometry() {
    if (fix_flag_) {
        x_ = gnss_x_;
        y_ = gnss_y_;
        double dgx = x_ - pre_gnss_x_;
        double dgy = y_ - pre_gnss_y_;
        double dist_gnss = std::hypot(dgx, dgy);

        if (vx_ > 0.2 && dist_gnss > 0.03 && dist_gnss < 3.0) {
            temp_gnss_yaw_ = std::atan2(dgy, dgx);
            
            // 角度の正規化（よりシンプルなfmodを使用）
            temp_gnss_yaw_ = std::fmod(temp_gnss_yaw_, 2 * M_PI);
            pre_yaw_ = std::fmod(pre_yaw_, 2 * M_PI);

            if (std::abs(std::abs(pre_yaw_) - std::abs(temp_gnss_yaw_)) > M_PI / 10) {
                yaw_ += diff_odrive_yaw_;
            } else {
                yaw_ = temp_gnss_yaw_;
            }
        } else {
            yaw_ += diff_odrive_yaw_;
        }

        if (gnss_count_ % 20 == 0) {
            pre_gnss_x_ = x_;
            pre_gnss_y_ = y_;
        }
        pre_yaw_ = temp_gnss_yaw_;
        gnss_count_++;
        switch_flag_ = true;
    } else {
        if (switch_flag_) {
            switch_yaw_ = yaw_ - odrive_yaw_;
        }
        auto [tx, ty] = transform_to_robot_frame(diff_odrive_x_, diff_odrive_y_, switch_yaw_);
        x_ += tx;
        y_ += ty;
        yaw_ += diff_odrive_yaw_;
        switch_flag_ = false;
    }

    FusedOdom result;
    result.x = x_;
    result.y = y_;
    result.yaw = yaw_;
    result.vx = vx_;
    result.vth = vth_;
    return result;
}

std::pair<double, double> GnssComponent::transform_to_robot_frame(double trans_x, double trans_y, double trans_yaw) {
    double x_in_robot = trans_x * cos(trans_yaw) - trans_y * sin(trans_yaw);
    double y_in_robot = trans_y * cos(trans_yaw) + trans_x * sin(trans_yaw);
    return {x_in_robot, y_in_robot};
}

} // namespace gnss