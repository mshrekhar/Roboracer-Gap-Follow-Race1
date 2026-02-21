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
    ReactiveFollowGap() : Node("reactive_follow_gap")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ReactiveFollowGap::scan_callback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);

        bubble_radius_ = 0.27;
        max_range_ = 3.0;

        max_speed_ = 3.0;
        min_speed_ = 0.8;

        fov_limit_ = 1.5708;  // 90° left/right

        RCLCPP_INFO(this->get_logger(), "Reactive Follow-The-Gap initialized");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    double bubble_radius_;
    double max_range_;
    double max_speed_;
    double min_speed_;
    double fov_limit_;

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
            int bubble_end = std::min(end_idx, closest_idx + bubble_size);

            for (int i = bubble_start; i <= bubble_end; ++i)
                ranges[i] = 0.0;
        }

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

        // ---------- Select best gap ----------
        Gap best_gap = gaps[0];
        for (size_t i = 1; i < gaps.size(); ++i)
        {
            if (gaps[i].max_depth > best_gap.max_depth + 0.5)
                best_gap = gaps[i];
            else if (std::abs(gaps[i].max_depth - best_gap.max_depth) <= 0.5)
                if (gaps[i].len > best_gap.len)
                    best_gap = gaps[i];
        }

        // =====================================================
        // 🔎 Detect LEFT TURN
        // =====================================================

        float left_sum = 0.0, right_sum = 0.0;
        int left_count = 0, right_count = 0;

        int mid_idx = (start_idx + end_idx) / 2;

        for (int i = start_idx; i < mid_idx; ++i) {
            left_sum += ranges[i];
            left_count++;
        }

        for (int i = mid_idx; i <= end_idx; ++i) {
            right_sum += ranges[i];
            right_count++;
        }

        float left_avg = left_sum / std::max(1, left_count);
        float right_avg = right_sum / std::max(1, right_count);

        bool left_turn_detected = (left_avg > right_avg + 0.5);

        // =====================================================
        // 🎯 Best point selection (Hybrid)
        // =====================================================

        int best_idx;

        if (left_turn_detected)
        {
            // Use geometric center to avoid wall hugging
            best_idx = (best_gap.start + best_gap.end) / 2;
        }
        else
        {
            // Use deep-cluster logic (smooth behavior)
            int deep_start = -1, deep_end = -1;

            for (int i = best_gap.start; i <= best_gap.end; ++i)
            {
                if (ranges[i] >= best_gap.max_depth - 0.15)
                {
                    if (deep_start == -1) deep_start = i;
                    deep_end = i;
                }
            }

            if (deep_start == -1) {
                deep_start = best_gap.start;
                deep_end = best_gap.end;
            }

            best_idx = (deep_start + deep_end) / 2;
        }

        // ---------- Steering ----------
        double steering_angle = angle_min + best_idx * angle_inc;
        double abs_angle = std::abs(steering_angle);
        double speed = max_speed_;

        if (abs_angle < 0.17)
            speed = max_speed_;
        else if (abs_angle < 0.35)
            speed = 2.0;
        else
            speed = min_speed_;

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