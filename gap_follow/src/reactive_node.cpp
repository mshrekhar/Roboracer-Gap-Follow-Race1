#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class ReactiveFollowGap : public rclcpp::Node {

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10,
            std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));

        drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, 10);

        car_width            = 0.35f;   // m
        car_length           = 0.5f;    // m
        bubble_radius        = 0.4f;    // m
        disparity_thresh     = 2.0f;    // m
        max_considered_range = 4.0f;    // m
        max_speed            = 3.0f;    // m/s
        min_speed            = 1.0f;    // m/s;
        steering_gain        = 1.0f;
        last_n               = 0;

        last_angle_min       = 0.0f;
        last_angle_inc       = 0.0f;

        last_steering_angle  = 0.0f;
        steering_initialized = false;

        RCLCPP_INFO(this->get_logger(),
                    "ReactiveFollowGap (disparity extender + corridor check) initialized");
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic     = "/drive";

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;

    float car_width;
    float car_length;
    float bubble_radius;
    float disparity_thresh;
    float max_considered_range;
    float max_speed;
    float min_speed;
    float steering_gain;
    int   last_n;

    float last_angle_min;
    float last_angle_inc;

    float last_steering_angle;
    bool  steering_initialized;

    // ---------------- cheap straight-corridor check ----------------
    bool is_corridor_free(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg,
                          float steering_angle,
                          float required_length)
    {
        const float side_margin = 0.05f;   // tune
        float half_width = 0.5f * (car_width + 2.0f * side_margin);

        float half_span = std::atan2(half_width, required_length);

        float corridor_min_angle = steering_angle - half_span;
        float corridor_max_angle = steering_angle + half_span;

        int   N         = static_cast<int>(scan_msg->ranges.size());
        float angle_min = scan_msg->angle_min;
        float angle_inc = scan_msg->angle_increment;

        for (int i = 0; i < N; ++i) {
            float angle = angle_min + i * angle_inc;
            if (angle < corridor_min_angle || angle > corridor_max_angle)
                continue;

            float r = scan_msg->ranges[i];
            if (!std::isfinite(r) || r <= 0.0f)
                continue;

            if (r < required_length) {
                return false;
            }
        }
        return true;
    }

    // ---------------- preprocess ----------------
    void preprocess_lidar(float* ranges)
    {
        if (last_n <= 0) return;

        const int   window   = 3;
        const float max_keep = 3.0f;

        std::vector<float> tmp(last_n);
        for (int i = 0; i < last_n; ++i) tmp[i] = ranges[i];

        for (int i = 0; i < last_n; ++i) {
            int s = std::max(0, i - 1);
            int e = std::min(last_n - 1, i + 1);

            float sum = 0.0f;
            int   cnt = 0;
            for (int j = s; j <= e; ++j) {
                float r = tmp[j];
                if (!std::isfinite(r) || r <= 0.0f) continue;
                if (r > max_keep) r = max_keep;
                sum += r;
                ++cnt;
            }

            if (cnt > 0) {
                ranges[i] = sum / static_cast<float>(cnt);
            } else {
                ranges[i] = tmp[i];
            }
        }
    }

    // ---------------- disparity extender ----------------
    void apply_disparity_extender(std::vector<float> &ranges,
                                  float angle_min,
                                  float angle_increment)
    {
        int n = static_cast<int>(ranges.size());
        if (n <= 1) return;

        const float max_ext_angle = 1.05f;  // ~±60°

        for (int i = 0; i < n - 1; ++i) {
            float d1 = ranges[i];
            float d2 = ranges[i + 1];
            if (!std::isfinite(d1) || !std::isfinite(d2) ||
                d1 <= 0.0f || d2 <= 0.0f)
                continue;

            float angle_i = angle_min + i * angle_increment;
            if (angle_i < -max_ext_angle || angle_i > max_ext_angle)
                continue;

            float diff = std::fabs(d1 - d2);
            if (diff < disparity_thresh) continue;

            int   close_idx = (d1 < d2) ? i : i + 1;
            float d_close   = std::max(ranges[close_idx], 0.01f);

            float effective_half_width = 1.0f * car_width;  // full width
            float theta = std::atan2(effective_half_width, d_close);
            int extend = std::max(1, static_cast<int>(std::ceil(theta / angle_increment)));

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

    // ---------------- max gap ----------------
    void find_max_gap(float* ranges, int* indice)
    {
        if (last_n <= 0) {
            indice[0] = 0;
            indice[1] = -1;
            return;
        }

        int best_start = 0;
        int best_end   = -1;
        int best_len   = 0;

        int cur_start = -1;

        for (int i = 0; i < last_n; ++i) {
            if (ranges[i] > 0.0f) {
                if (cur_start == -1) cur_start = i;
            } else {
                if (cur_start != -1) {
                    int cur_end = i - 1;
                    int cur_len = cur_end - cur_start + 1;
                    if (cur_len > best_len) {
                        best_len   = cur_len;
                        best_start = cur_start;
                        best_end   = cur_end;
                    }
                    cur_start = -1;
                }
            }
        }

        if (cur_start != -1) {
            int cur_end = last_n - 1;
            int cur_len = cur_end - cur_start + 1;
            if (cur_len > best_len) {
                best_len   = cur_len;
                best_start = cur_start;
                best_end   = cur_end;
            }
        }

        indice[0] = best_start;
        indice[1] = best_end;
    }

    int find_best_point(float* ranges, int* indice)
    {
        int start = indice[0];
        int end   = indice[1];
        if (end < start || last_n <= 0) return last_n / 2;

        const int   min_half_width_indices = 3;
        const float deep_thresh = 1.2f;
        const float deep_band   = 0.8f;

        auto is_wide_enough = [&](int idx) {
            if (idx < start || idx > end) return false;
            int left  = std::max(start, idx - min_half_width_indices);
            int right = std::min(end,   idx + min_half_width_indices);
            for (int j = left; j <= right; ++j) {
                if (ranges[j] <= 0.0f) return false;
            }
            return true;
        };

        float deepest = -1.0f;
        for (int i = start; i <= end; ++i) {
            float d = ranges[i];
            if (d > deepest) deepest = d;
        }

        if (deepest > deep_thresh) {
            float lower = deepest - deep_band;
            if (lower < 0.0f) lower = 0.0f;

            // Collect all "deep" indices
            std::vector<int> deep_idxs;
            for (int i = start; i <= end; ++i) {
                float d = ranges[i];
                if (d >= lower && d > 0.0f && is_wide_enough(i)) {
                    deep_idxs.push_back(i);
                }
            }

            if (!deep_idxs.empty()) {
                // Instead of exact center, bias toward front (more straight)
                int deep_start = deep_idxs.front();
                int deep_end   = deep_idxs.back();

                // front_bias in [0,1]: 0.0 = pure center, 1.0 = toward far end
                const float front_bias = 0.3f;   // tune this

                int mid = static_cast<int>(
                    std::round((1.0f - front_bias) * 0.5f * (deep_start + deep_end)
                            + front_bias * deep_end)
                );

                if (!is_wide_enough(mid)) {
                    mid = (deep_start + deep_end) / 2;
                }
                return mid;
            }
        }

        // 3) fallback: deepest wide-enough point
        int   best_idx = start;
        float best_d   = -1.0f;
        for (int i = start; i <= end; ++i) {
            float d = ranges[i];
            if (d <= 0.0f) continue;
            if (!is_wide_enough(i)) continue;

            if (d > best_d) {
                best_d = d;
                best_idx = i;
            }
        }

        if (best_d < 0.0f) {
            best_idx = (start + end) / 2;
        }

        return best_idx;
    }


    // ---------------- main callback ----------------
    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        last_angle_min = scan_msg->angle_min;
        last_angle_inc = scan_msg->angle_increment;

        std::vector<float> ranges_vec = scan_msg->ranges;
        last_n = static_cast<int>(ranges_vec.size());
        if (last_n == 0) return;

        // 0) Restrict to [-120°, +120°]
        const float use_min_angle = -2.0944f;
        const float use_max_angle =  2.0944f;
        for (int i = 0; i < last_n; ++i) {
            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            if (angle < use_min_angle || angle > use_max_angle) {
                ranges_vec[i] = 0.0f;
            }
        }

        // 1) Clean & clamp
        for (int i = 0; i < last_n; ++i) {
            float &r = ranges_vec[i];
            if (!std::isfinite(r) || r <= 0.0f) {
                r = 0.0f;
            } else if (r > max_considered_range) {
                r = max_considered_range;
            }
        }

        // 2) Closest point
        int   closest_idx = -1;
        float closest_d   = 1e9f;
        for (int i = 0; i < last_n; ++i) {
            float d = ranges_vec[i];
            if (d > 0.0f && d < closest_d) {
                closest_d = d;
                closest_idx = i;
            }
        }

        // 3) Bubble
        if (closest_idx >= 0 && std::isfinite(closest_d)) {
            float angle_inc = scan_msg->angle_increment;
            float theta = std::atan2(bubble_radius, std::max(closest_d, 0.01f));
            int bubble_idx = std::max(1, static_cast<int>(std::ceil(theta / angle_inc)));

            int start = std::max(0, closest_idx - bubble_idx);
            int end   = std::min(last_n - 1, closest_idx + bubble_idx);
            for (int i = start; i <= end; ++i)
                ranges_vec[i] = 0.0f;
        }

        // 4) Disparity extender
        apply_disparity_extender(ranges_vec,
                                 scan_msg->angle_min,
                                 scan_msg->angle_increment);

        // 5) Preprocess
        std::vector<float> working = ranges_vec;
        preprocess_lidar(working.data());

        // 6) Max gap
        int gap[2] = {0, 0};
        find_max_gap(working.data(), gap);

        // 7) Best point
        int best_idx = find_best_point(working.data(), gap);

        float steering_angle =
            scan_msg->angle_min + best_idx * scan_msg->angle_increment;

        // 7b) Front 180° side safety
        {
            const float front_side_safe_dist = 0.5f;
            const float left_min  = 0.0f;
            const float left_max  = 1.57f;
            const float right_min = -1.57f;
            const float right_max = 0.0f;

            const auto &raw_ranges = scan_msg->ranges;
            int N = static_cast<int>(raw_ranges.size());

            auto has_close_obstacle_front_side = [&](bool left_side) -> bool {
                for (int i = 0; i < N; ++i) {
                    float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                    float r = raw_ranges[i];
                    if (!std::isfinite(r) || r <= 0.0f) continue;

                    if (left_side) {
                        if (angle >= left_min && angle <= left_max &&
                            r < front_side_safe_dist) {
                            return true;
                        }
                    } else {
                        if (angle >= right_min && angle <= right_max &&
                            r < front_side_safe_dist) {
                            return true;
                        }
                    }
                }
                return false;
            };

            if (steering_angle > 0.0f) {
                if (has_close_obstacle_front_side(true)) {
                    steering_angle = 0.0f;
                }
            } else if (steering_angle < 0.0f) {
                if (has_close_obstacle_front_side(false)) {
                    steering_angle = 0.0f;
                }
            }
        }

        // 7c) Speed from steering magnitude
        float abs_angle = std::fabs(steering_angle);
        float speed;
        if (abs_angle < 0.15f) {
            speed = max_speed;
        } else if (abs_angle < 0.4f) {
            speed = 0.5f * (max_speed + min_speed);
        } else {
            speed = min_speed;
        }

        // 7d) Straight-corridor requirement: need at least 1 car length
        const float required_len = car_length;

        if (!is_corridor_free(scan_msg, steering_angle, required_len)) {
            float straight_angle = 0.0f;
            if (is_corridor_free(scan_msg, straight_angle, required_len)) {
                steering_angle = straight_angle;
            } else {
                speed = min_speed;
            }
        }

        // 7e) Minimal smoothing (optional)
        if (!steering_initialized) {
            last_steering_angle = steering_angle;
            steering_initialized = true;
        } else {
            const float alpha = 0.2f;
            steering_angle = alpha * last_steering_angle + (1.0f - alpha) * steering_angle;
            last_steering_angle = steering_angle;
        }

        // 8) Publish drive
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "laser";
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = speed;
        drive_pub->publish(drive_msg);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
