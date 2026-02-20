#include "ondrive.hpp"
#include "ondrive/utils/visualize.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

namespace {
    std::vector<datapod::Point> build_carrot_ondrives() {
        return {{-2.0f, -6.0f}, {-0.5f, -5.0f}, {1.0f, -4.0f},  {2.5f, -3.0f},
                {4.0f, -1.5f},  {5.5f, -0.5f},  {6.0f, -0.25f}, {6.0f, -3.0f}};
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
    auto rec = std::make_shared<rerun::RecordingStream>("ondrive_carrot_demo", "carrot");
    if (rec->connect_grpc("rerun+http://0.0.0.0:9876/proxy").is_err()) {
        std::cerr << "Failed to connect to rerun\n";
        return 1;
    }

    rec->log("", rerun::Clear::RECURSIVE);
    rec->log_with_static("", true, rerun::Clear::RECURSIVE);

    std::cout << "Visualization initialized for Carrot demo\n";

    ondrive::Tracker navigator(ondrive::TrackerType::CARROT);

    auto params = navigator.get_controller_params();
    params.carrot_distance = 1.2f;
    params.linear_kp = 1.2f;
    params.angular_kp = 1.0f;
    navigator.set_controller_params(params);

    ondrive::RobotConstraints constraints;
    constraints.max_linear_velocity = 0.3;
    constraints.max_angular_velocity = 0.8;
    constraints.wheelbase = 0.4;

    navigator.init(constraints, rec);

    const auto ondrives = build_carrot_ondrives();
    if (ondrives.empty()) {
        std::cerr << "No carrot ondrives defined\n";
        return 1;
    }

    ondrive::visualize::show_path(rec, make_visual_path(ondrives), "carrot_path", rerun::Color(255, 140, 0));

    auto set_navigation_goal = [&](size_t index) {
        ondrive::NavigationGoal next_goal(ondrives[index], 0.25f, 0.3f);
        navigator.set_goal(next_goal);

        ondrive::Goal viz_goal;
        viz_goal.target_pose = datapod::Pose{next_goal.target, datapod::Quaternion::from_euler(0.0, 0.0, 0.0)};
        viz_goal.tolerance_position = next_goal.tolerance;
        ondrive::visualize::show_goal(rec, viz_goal, "carrot_goal", rerun::Color(255, 140, 0));
    };

    size_t ondrive_index = 0;
    set_navigation_goal(ondrive_index);

    ondrive::RobotState robot_state;
    robot_state.pose.point = datapod::Point{-2.0, -6.0};
    robot_state.pose.rotation = datapod::Quaternion::from_euler(0.0, 0.0, 0.5f);

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

            ondrive::visualize::show_robot_state(rec, robot_state, "robot_carrot", rerun::Color(255, 165, 0));
            navigator.tock();

            if (current_time - last_print_time >= print_interval) {
                std::cout << "Carrot follower chasing goal...\n";
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
        std::cout << "Carrot ondrive path completed!\n";
    } else {
        std::cerr << "Carrot demo timed out before reaching the final ondrive\n";
    }

    return 0;
}
