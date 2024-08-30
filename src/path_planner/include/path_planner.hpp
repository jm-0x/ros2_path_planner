#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <concepts>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lfs_msgs/msg/map.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

// Concept to enforce the requirements for a Path Planning Algorithm
template <typename T>
concept path_planning_algorithm =
    requires(T t, const std::vector<double> &x, const std::vector<double> &y) {
      {
        t.generate_path(x, y)
      } -> std::convertible_to<std::vector<geometry_msgs::msg::PoseStamped>>;
    };

template <path_planning_algorithm Algorithm> class path_planner {
public:
  path_planner() = default;

  void compute_path(const lfs_msgs::msg::Map::SharedPtr &map_msg,
                    nav_msgs::msg::Path &path) {
    if (map_msg->x.size() != map_msg->y.size() ||
        map_msg->x.size() != map_msg->color.size()) {
      RCLCPP_ERROR(
          rclcpp::get_logger("PathPlanner"),
          "Inconsistent sizes of x, y, and color arrays in Map message.");
      return;
    }

    std::vector<double> x = map_msg->x;
    std::vector<double> y = map_msg->y;

    Algorithm algorithm;
    auto path_poses = algorithm.generate_path(x, y);

    path.header.frame_id = "map";
    path.header.stamp = rclcpp::Clock().now();
    path.poses = std::move(path_poses);
  }
};

#endif // PATH_PLANNER_HPP
