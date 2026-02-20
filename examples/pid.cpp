#include "ondrive.hpp"
#include "ondrive/utils/visualize.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

namespace {
    std::vector<datapod::Point> build_pid_ondrives() {
        return {{0.0f, 0.0f}, {1.5f, 0.5f}, {3.0f, 1.5f}, {4.5f, 2.0f}, {6.0f, 3.2f}, {7.0f, 3.8f}, {8.0f, 4.0f}};
    }

    ondrive::Path make_visual_path(const std::vector<datapod::Point> &points) {
        ondrive::Path path;
        for (const auto &pt : points) {
            ondrive::Pose pose;
            pose.point = pt;
            pose.rotation = datapod::Quaternion::from_euler(0.0, 0.0, 0.0);
            path.ondrives.push_back(pose);
        }
        return path;
    }
} // namespace

int main() {
    auto rec = std::make_shared<rerun::RecordingStream>("ondrive_pid_demo", "pid");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    rec->log("", rerun::Clear::RECURSIVE);
    rec->log_with_static("", true, rerun::Clear::RECURSIVE);

    std::cout << "Visualization initialized for PID demo\n";

    ondrive::Tracker navigator(ondrive::TrackerType::PID);

    auto params = navigator.get_controller_params();
    params.linear_kp = 2.5f;
    params.angular_kp = 1.8f;
    params.angular_kd = 0.2f;
    navigator.set_controller_params(params);

    ondrive::RobotConstraints constraints;
    constraints.max_linear_velocity = 0.35;
    constraints.max_angular_velocity = 1.0;
    constraints.wheelbase = 0.45;

    navigator.init(constraints, rec);

    const auto ondrives = build_pid_ondrives();
    if (ondrives.empty()) {
        std::cerr << "No PID ondrives defined\n";
        return 1;
    }

    ondrive::visualize::show_path(rec, make_visual_path(ondrives), "pid_path", rerun::Color(0, 120, 255));

    auto set_navigation_goal = [&](size_t index) {
        ondrive::NavigationGoal next_goal(ondrives[index], 0.2f, 0.35f);
        navigator.set_goal(next_goal);

        ondrive::Goal viz_goal;
        viz_goal.target_pose = datapod::Pose{next_goal.target, datapod::Quaternion::from_euler(0.0, 0.0, 0.0)};
        viz_goal.tolerance_position = next_goal.tolerance;
        ondrive::visualize::show_goal(rec, viz_goal, "pid_goal");
    };

    size_t ondrive_index = 0;
    set_navigation_goal(ondrive_index);

    ondrive::RobotState robot_state;
    robot_state.pose.point = datapod::Point{0.0, 0.0};
    robot_state.pose.rotation = datapod::Quaternion::from_euler(0.0, 0.0, 0.0);

    float dt = 0.05f;
    float current_time = 0.0f;
    float last_print_time = 0.0f;
    const float print_interval = 0.5f;

    for (int i = 0; i < 2000 && ondrive_index < ondrives.size(); ++i) {
        auto cmd = navigator.tick(robot_state, dt);

        if (cmd.valid) {
            robot_state.velocity.linear = cmd.linear_velocity;
            robot_state.velocity.angular = cmd.angular_velocity;

            robot_state.pose.point.x += cmd.linear_velocity * std::cos(robot_state.pose.rotation.to_euler().yaw) * dt;
            robot_state.pose.point.y += cmd.linear_velocity * std::sin(robot_state.pose.rotation.to_euler().yaw) * dt;
            robot_state.pose.rotation = datapod::Quaternion::from_euler(
                0.0, 0.0, robot_state.pose.rotation.to_euler().yaw + cmd.angular_velocity * dt);

            current_time += dt;

            ondrive::visualize::show_robot_state(rec, robot_state, "robot_pid", rerun::Color(0, 120, 255));
            navigator.tock();

            if (current_time - last_print_time >= print_interval) {
                std::cout << "PID controller driving toward goal...\n";
                last_print_time = current_time;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }

        if (navigator.is_goal_reached()) {
            ondrive_index++;
            if (ondrive_index < ondrives.size()) {
                set_navigation_goal(ondrive_index);
            }
        }
    }

    if (ondrive_index >= ondrives.size()) {
        std::cout << "PID ondrive path completed!\n";
    } else {
        std::cerr << "PID demo timed out before completing the ondrive path\n";
    }

    return 0;
}
