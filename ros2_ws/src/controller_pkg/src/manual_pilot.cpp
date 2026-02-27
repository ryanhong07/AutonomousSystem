#include <memory>
#include <chrono>
#include <cmath>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory.hpp"

using namespace std::chrono_literals;

enum State { NAV_TO_ENTRANCE, SEARCHING_CAVE, HUNTING, LOGGING, CLEARING_ZONE, MISSION_COMPLETE };

struct Waypoint { double x; double y; double z; };

class ManualPilot : public rclcpp::Node {
public:
    ManualPilot() : Node("manual_pilot") {
        target_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/lantern/target_data", 10, std::bind(&ManualPilot::auto_steer_callback, this, std::placeholders::_1));
        traj_pub_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>("/command/trajectory", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&ManualPilot::publish_traj, this));

        tx_ = -36.0; ty_ = 10.0; tz_ = 10.0; current_yaw_ = 3.14159;
        current_state_ = NAV_TO_ENTRANCE;
        start_time_ = this->now();
        entrance_ = {-315.14, 8.38, 14.99};
    }

private:
    double clearing_target_z_ = 0.0;
    std::vector<Waypoint> found_lanterns_;

    void change_state(State new_state, const std::string& action_msg) {
        if (current_state_ != new_state) {
            current_state_ = new_state;
            RCLCPP_INFO(this->get_logger(), ">>> %s", action_msg.c_str());
        }
    }

    void auto_steer_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        if ((this->now() - start_time_).seconds() < 2.0) return;
        if (current_state_ == LOGGING || current_state_ == CLEARING_ZONE || current_state_ == MISSION_COMPLETE) return;

        // Mode 2.0: Exploration (Fine-tuned for smoothness)
        if (msg->z == 2.0 && current_state_ == SEARCHING_CAVE) {
            // Reduced from 0.012 to 0.008 for gentler turns
            current_yaw_ -= msg->x * 0.008; 
            // Reduced from 0.01 to 0.005 for stable altitude
            tz_ += msg->y * 0.005; 
            return;
        }

        // Mode 1.0: Hunting
        if (msg->z != 2.0 && msg->z != 0.0) { 
            if (current_state_ == SEARCHING_CAVE) change_state(HUNTING, "Lantern Spotted.");

            if (current_state_ == HUNTING) {
                double stop_threshold = 800000.0; 
                // Reduced from 0.0015 to 0.001 for smooth centering
                current_yaw_ -= msg->x * 0.001; 

                if (msg->y < stop_threshold) {
                    tx_ += std::cos(current_yaw_) * 0.12;
                    ty_ += std::sin(current_yaw_) * 0.12;
                    tz_ -= msg->z * 0.008;
                } else {
                    reach_time_ = this->now();
                    found_lanterns_.push_back({tx_, ty_, tz_});
                    RCLCPP_INFO(this->get_logger(), "Lantern #%ld Found!", found_lanterns_.size());
                    
                    clearing_target_z_ = tz_ + 10.0;
                    change_state(LOGGING, "LOGGING: Holding position for 2s.");
                }
            }
        } 
        else if (current_state_ == HUNTING) {
            change_state(SEARCHING_CAVE, "Visual lost. Gentler recovery.");
        }
    }

    void publish_traj() {
        if (current_state_ == NAV_TO_ENTRANCE) {
            double angle = std::atan2(entrance_.y - ty_, entrance_.x - tx_);
            current_yaw_ = angle;
            tx_ += std::cos(current_yaw_) * 0.20; // Reduced transit speed for stability
            ty_ += std::sin(current_yaw_) * 0.20;
            if (tz_ < entrance_.z) tz_ += 0.08;
            if (std::sqrt(std::pow(entrance_.x-tx_,2)+std::pow(entrance_.y-ty_,2)) < 3.0) 
                change_state(SEARCHING_CAVE, "EXPLORING.");
        }
        else if (current_state_ == SEARCHING_CAVE) {
            // Reduced from 0.12 to 0.10 to help computer lag
            tx_ += std::cos(current_yaw_) * 0.10;
            ty_ += std::sin(current_yaw_) * 0.10;
        }
        else if (current_state_ == LOGGING) {
            if ((this->now() - reach_time_).seconds() >= 2.0) {
                if (found_lanterns_.size() >= 4) {
                    change_state(MISSION_COMPLETE, "MISSION SUCCESS: All 4 logged.");
                } else {
                    change_state(CLEARING_ZONE, "EVADING: STEP 1 - Vertical Climb.");
                    reach_time_ = this->now();
                }
            }
        }
        else if (current_state_ == CLEARING_ZONE) {
            if (tz_ < (clearing_target_z_ - 0.1)) {
                tz_ += 0.4; // Slightly slower climb rate for stability
                reach_time_ = this->now(); 
            } else {
                // Reduced from 0.15 to 0.12 for smoother recovery
                tx_ += std::cos(current_yaw_) * 0.12; 
                ty_ += std::sin(current_yaw_) * 0.12;

                if ((this->now() - reach_time_).seconds() >= 10.0) {
                    change_state(SEARCHING_CAVE, "RESUMING SEARCH.");
                }
            }
        }
        else if (current_state_ == MISSION_COMPLETE) {
            static bool summary_printed = false;
            if (!summary_printed) {
                RCLCPP_INFO(this->get_logger(), "========= LANTERN SUMMARY =========");
                for(size_t i=0; i < found_lanterns_.size(); ++i) {
                    RCLCPP_INFO(this->get_logger(), "Lantern %ld: [X: %.2f, Y: %.2f, Z: %.2f]", 
                                i+1, found_lanterns_[i].x, found_lanterns_[i].y, found_lanterns_[i].z);
                }
                RCLCPP_INFO(this->get_logger(), "==================================");
                summary_printed = true;
            }
        }
        send_command();
    }

    void send_command() {
        auto message = trajectory_msgs::msg::MultiDOFJointTrajectory();
        message.header.stamp = this->now();
        message.header.frame_id = "world";
        trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
        geometry_msgs::msg::Transform transform;
        transform.translation.x = tx_; transform.translation.y = ty_; transform.translation.z = tz_;
        transform.rotation.z = std::sin(current_yaw_ / 2.0); transform.rotation.w = std::cos(current_yaw_ / 2.0);
        point.transforms.push_back(transform);
        point.velocities.push_back(geometry_msgs::msg::Twist());
        point.accelerations.push_back(geometry_msgs::msg::Twist());
        point.time_from_start.nanosec = 100000000;
        message.points.push_back(point);
        traj_pub_->publish(message);
    }

    Waypoint entrance_; State current_state_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr target_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double tx_, ty_, tz_, current_yaw_; rclcpp::Time reach_time_, start_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManualPilot>());
    rclcpp::shutdown();
    return 0;
}
