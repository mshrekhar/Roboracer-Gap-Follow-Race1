#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

struct Gap {
    int start;
    int end;
    int len;
    float max_depth;
};

class ReactiveFollowGap : public rclcpp::Node
{
public:
    ReactiveFollowGap() : Node("reactive_follow_gap"),
                          prev_error_(0.0),
                          pd_initialized_(false),
                          prev_speed_(0.0),
                          speed_init_(false)
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ReactiveFollowGap::scan_callback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);

        bubble_radius_ = 0.58;
        max_range_ = 5.0;

        max_speed_ = 4.0;
        min_speed_ = 0.8;

        fov_limit_ = 1.5708;  // 90° left/right

        // PD gains (tune)
        Kp_ = 0.5;
        Kd_ = 0.09;

        RCLCPP_INFO(this->get_logger(),
                    "Reactive Follow-The-Gap with corridor check initialized");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    double bubble_radius_;
    double max_range_;
    double max_speed_;
    double min_speed_;
    double fov_limit_;

    // PD state
    double Kp_;
    double Kd_;
    double prev_error_;
    bool   pd_initialized_;

    // Speed smoothing state
    double prev_speed_;
    bool   speed_init_;

    // ---------------- disparity extender ----------------
    void apply_disparity_extender(std::vector<float> &ranges,
                                  double angle_min,
                                  double angle_inc)
    {
        int n = static_cast<int>(ranges.size());
        if (n <= 1) return;

        const double disparity_thresh = 1.0;   // m, tune
        const double car_width        = 0.3;   // m, tune
        const double max_ext_angle    = 1.0;   // ~±57 deg, tune

        for (int i = 0; i < n - 1; ++i) {
            float d1 = ranges[i];
            float d2 = ranges[i + 1];
            if (!std::isfinite(d1) || !std::isfinite(d2) ||
                d1 <= 0.0f || d2 <= 0.0f)
                continue;

            double angle_i = angle_min + i * angle_inc;
            if (angle_i < -max_ext_angle || angle_i > max_ext_angle)
                continue;

            double diff = std::fabs(d1 - d2);
            if (diff < disparity_thresh)
                continue;

            int   close_idx = (d1 < d2) ? i : i + 1;
            float d_close   = std::max(ranges[close_idx], 0.01f);

            double theta = std::atan2(car_width, d_close);
            int extend = std::max(1, static_cast<int>(std::ceil(theta / angle_inc)));
            int max_extend_indices = 15;
            if (extend > max_extend_indices) extend = max_extend_indices;

            if (close_idx == i) {
                int start = i + 1;
                int end   = std::min(n - 1, start + extend);
                for (int j = start; j <= end; ++j)
                    if (ranges[j] > d_close) ranges[j] = d_close;
            } else {
                int end   = i;
                int start = std::max(0, end - extend);
                for (int j = end; j >= start; --j)
                    if (ranges[j] > d_close) ranges[j] = d_close;
            }
        }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
    {
        std::vector<float> ranges = msg->ranges;
        double angle_min = msg->angle_min;
        double angle_inc = msg->angle_increment;

        int start_idx = std::max(0, static_cast<int>((-fov_limit_ - angle_min) / angle_inc));
        int end_idx = std::min(static_cast<int>(ranges.size() - 1),
                               static_cast<int>((fov_limit_ - angle_min) / angle_inc));

        // ---------- Preprocess ----------
        for (int i = start_idx; i <= end_idx; ++i)
        {
            if (!std::isfinite(ranges[i])) {
                ranges[i] = max_range_;
            } else if (ranges[i] > max_range_) {
                ranges[i] = max_range_;
            } else if (ranges[i] <= 0.0) {
                ranges[i] = 0.0;
            }
        }

        // ---------- Closest obstacle ----------
        float min_dist = 1e9;
        int closest_idx = -1;

        for (int i = start_idx; i <= end_idx; ++i)
        {
            if (ranges[i] > 0.01 && ranges[i] < min_dist)
            {
                min_dist = ranges[i];
                closest_idx = i;
            }
        }

        // ---------- Bubble ----------
        if (closest_idx != -1)
        {
            double theta = std::asin(
                std::min(1.0, bubble_radius_ / std::max<double>(min_dist, 0.01)));

            int bubble_size = std::ceil(theta / angle_inc);

            int bubble_start = std::max(start_idx, closest_idx - bubble_size);
            int bubble_end   = std::min(end_idx, closest_idx + bubble_size);

            for (int i = bubble_start; i <= bubble_end; ++i)
                ranges[i] = 0.0;
        }

        // ---------- Forward corridor check (on bubble-filtered data) ----------
        // Define a narrower front sector for corridor detection (e.g. ±60°)
        double corridor_fov = M_PI / 3.0;  // 60 deg
        int corr_start = std::max(start_idx,
                                  static_cast<int>((-corridor_fov - angle_min) / angle_inc));
        int corr_end   = std::min(end_idx,
                                  static_cast<int>(( corridor_fov - angle_min) / angle_inc));

        // Center band for "front_min"
        double front_band = 20.0 * M_PI / 180.0; // ±20 deg
        int front_start = std::max(start_idx,
                                   static_cast<int>((-front_band - angle_min) / angle_inc));
        int front_end   = std::min(end_idx,
                                   static_cast<int>(( front_band - angle_min) / angle_inc));

        float front_min = max_range_;
        float left_sum = 0.0f, right_sum = 0.0f;
        int left_cnt = 0, right_cnt = 0;

        for (int i = corr_start; i <= corr_end; ++i)
        {
            double ang = angle_min + i * angle_inc;
            float d = ranges[i];

            if (i >= front_start && i <= front_end && d > 0.01f)
                front_min = std::min(front_min, d);

            if (d > 0.01f) {
                if (ang < 0.0) {
                    left_sum += d;
                    left_cnt++;
                } else {
                    right_sum += d;
                    right_cnt++;
                }
            }
        }

        float left_mean  = (left_cnt  > 0) ? (left_sum  / left_cnt)  : 0.0f;
        float right_mean = (right_cnt > 0) ? (right_sum / right_cnt) : 0.0f;

        bool corridor_open = (front_min > 2.5f);                 // tune
        bool roughly_centered = (left_mean > 0.0f && right_mean > 0.0f &&
                                 std::fabs(left_mean - right_mean) < 0.5f); // tune

        // ---------- Disparity extender ----------
        apply_disparity_extender(ranges, angle_min, angle_inc);

        // ---------- Find gaps ----------
        std::vector<Gap> gaps;
        int current_start = -1;
        float current_max_depth = 0.0;

        for (int i = start_idx; i <= end_idx; ++i)
        {
            if (ranges[i] > 0.0)
            {
                if (current_start == -1) {
                    current_start = i;
                    current_max_depth = ranges[i];
                } else {
                    current_max_depth = std::max(current_max_depth, ranges[i]);
                }
            }
            else
            {
                if (current_start != -1)
                {
                    gaps.push_back(
                        {current_start, i - 1, i - current_start, current_max_depth});
                    current_start = -1;
                }
            }
        }

        if (current_start != -1)
            gaps.push_back({current_start, end_idx,
                            end_idx - current_start + 1, current_max_depth});

        if (gaps.empty()) return;

        // ---------- Select best gap (depth + width) ----------
        Gap best_gap = gaps[0];
        for (size_t i = 1; i < gaps.size(); ++i)
        {
            if (gaps[i].max_depth > best_gap.max_depth + 0.5f)
                best_gap = gaps[i];
            else if (std::abs(gaps[i].max_depth - best_gap.max_depth) <= 0.5f)
                if (gaps[i].len > best_gap.len)
                    best_gap = gaps[i];
        }

        // ---------- Best point selection (Center-first) ----------  n
        int gap_center_idx = (best_gap.start + best_gap.end) / 2;
        float gap_center_range = ranges[gap_center_idx];

        bool corridor_deep = (gap_center_range > 2.5f);  // tune

        int best_idx;
        if (corridor_deep)
        {
            best_idx = gap_center_idx;
        }
        else
        {
            int deep_start = -1, deep_end = -1;

            for (int i = best_gap.start; i <= best_gap.end; ++i)
            {
                if (ranges[i] >= best_gap.max_depth - 0.15f)
                {
                    if (deep_start == -1) deep_start = i;
                    deep_end = i;
                }
            }

            if (deep_start == -1) {
                deep_start = best_gap.start;
                deep_end   = best_gap.end;
            }

            int deep_center_idx = (deep_start + deep_end) / 2;

            int idx_diff = std::abs(deep_center_idx - gap_center_idx);
            int max_snap_diff = 5;  // tune

            if (idx_diff <= max_snap_diff) {
                best_idx = gap_center_idx;
            } else {
                best_idx = deep_center_idx;
            }
        }

        // ---------- Steering angle computation ----------
        double angle_target = angle_min + best_idx * angle_inc;
        double error = angle_target;  // desired center = 0

        double d_error = 0.0;
        if (!pd_initialized_) {
            d_error = 0.0;
            pd_initialized_ = true;
        } else {
            d_error = error - prev_error_;
        }
        prev_error_ = error;

        double steering_angle = Kp_ * error + Kd_ * d_error;

        // ---------- Global steering limit: ±50 degrees ----------
        double max_steer = 50.0 * M_PI / 180.0; // ~0.8727 rad
        if (steering_angle >  max_steer) steering_angle =  max_steer;
        if (steering_angle < -max_steer) steering_angle = -max_steer;

        // ---------- Corridor override: straight and fast ----------
        // If corridor is open and left/right distances are similar,
        // ignore gap-based steering and go (almost) straight.
        if (corridor_open && roughly_centered) {
            steering_angle = 0.0;
        }

        double abs_angle = std::abs(steering_angle);

        // ---------- Angle-based speed ----------
        double speed_angle;
        if (corridor_open && roughly_centered) {
            // straight corridor, centered -> max speed
            speed_angle = max_speed_;
        } else if (abs_angle < 0.10) {
            speed_angle = max_speed_;
        } else if (abs_angle < 0.20) {
            speed_angle = 0.8 * max_speed_;
        } else if (abs_angle < 0.30) {
            speed_angle = 0.6 * max_speed_;
        } else if (abs_angle < 0.45) {
            speed_angle = 0.4 * max_speed_;
            if (speed_angle < min_speed_)
                speed_angle = min_speed_;
        } else {
            speed_angle = min_speed_;
        }

        // ---------- Distance-based speed around best_idx ----------
        int window = 5;
        float min_ahead = max_range_;
        for (int i = std::max(best_idx - window, start_idx);
             i <= std::min(best_idx + window, end_idx); ++i)
        {
            if (ranges[i] > 0.01f)
                min_ahead = std::min(min_ahead, ranges[i]);
        }

        double speed_dist;
        if (min_ahead > 3.5)
            speed_dist = max_speed_;
        else if (min_ahead > 2.0)
            speed_dist = 2.0;
        else if (min_ahead > 1.2)
            speed_dist = 1.2;
        else
            speed_dist = min_speed_;

        double speed = std::min(speed_angle, speed_dist);

        // ---------- Optional global minimum-based cap ----------
        float global_min = 1e9;
        for (int i = start_idx; i <= end_idx; ++i)
        {
            if (ranges[i] > 0.01f)
                global_min = std::min(global_min, ranges[i]);
        }
        if (global_min < 0.7)
            speed = std::min(speed, 1.0);

        // ---------- Acceleration smoothing ----------
        if (!speed_init_) {
            prev_speed_ = speed;
            speed_init_ = true;
        } else {
            double max_accel = 1.0;   // m/s^2 up
            double max_decel = 2.0;   // m/s^2 down
            double dt = 1.0 / 15.0;   // assume ~20 Hz LiDAR

            double max_up   = max_accel * dt;
            double max_down = max_decel * dt;

            double dv = speed - prev_speed_;
            if (dv >  max_up)   dv =  max_up;
            if (dv < -max_down) dv = -max_down;

            speed = prev_speed_ + dv;
            prev_speed_ = speed;
        }

        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->now();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = speed;

        drive_pub_->publish(drive_msg);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
