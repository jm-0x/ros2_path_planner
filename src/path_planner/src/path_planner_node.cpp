#include "cubic_spline.hpp"
#include "path_planner.hpp"
#include <lfs_msgs/msg/map.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

class path_planner_node : public rclcpp::Node {
public:
  path_planner_node() : Node("path_planner_node") {
    map_subscription_ = this->create_subscription<lfs_msgs::msg::Map>(
        "/mapping/track_points", 10,
        [this](const lfs_msgs::msg::Map::SharedPtr msg){
            this->map_callback(msg);
            });

    path_publisher_ =
        this->create_publisher<nav_msgs::msg::Path>("/path/global", 10);
  }

private:
  void map_callback(const lfs_msgs::msg::Map::SharedPtr msg) {
    nav_msgs::msg::Path path;
    path_planner<cubic_spline> planner;
    planner.compute_path(msg, path);

    path_publisher_->publish(path);
  }

  rclcpp::Subscription<lfs_msgs::msg::Map>::SharedPtr map_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<path_planner_node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
