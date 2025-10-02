#pragma once

#include <cmath>
#include <utility>

namespace gnss {

// 計算結果をまとめて返すための構造体
struct FusedOdom {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double vx = 0.0;
    double vth = 0.0;
};

// オドメトリデータをまとめて渡すための構造体
struct OdomInput {
    double x;
    double y;
    double yaw;
    double vx;
    double vth;
};

class GnssComponent {
public:
    GnssComponent();

    // 外部からセンサーデータを設定するメソッド
    void setFixStatus(int status);
    void setGnssPose(double x, double y);
    void updateOdometry(const OdomInput& odom_data);

    // 現在の状態に基づいて統合された自己位置を計算するメソッド
    FusedOdom computeFusedOdometry();

private:
    std::pair<double, double> transform_to_robot_frame(double trans_x, double trans_y, double trans_yaw);

    // 元のコードのメンバー変数をここに集約
    double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;
    double gnss_x_ = 0.0, gnss_y_ = 0.0;
    double odrive_yaw_ = 0.0, vx_ = 0.0, vth_ = 0.0, temp_gnss_yaw_ = 0.0, switch_yaw_ = 0.0;
    double pre_odrive_x_ = 0.0, pre_odrive_y_ = 0.0, pre_odrive_yaw_ = 0.0;
    double diff_odrive_x_ = 0.0, diff_odrive_y_ = 0.0, diff_odrive_yaw_ = 0.0;
    double pre_gnss_x_ = 0.0, pre_gnss_y_ = 0.0, pre_yaw_ = 0.0;

    bool fix_flag_ = true, switch_flag_ = true;
    int gnss_count_ = 0;
};

} // namespace gnss
