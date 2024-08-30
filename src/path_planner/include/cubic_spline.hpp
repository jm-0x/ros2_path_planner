#ifndef CUBIC_SPLINE_HPP
#define CUBIC_SPLINE_HPP

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

class cubic_spline {
public:
  cubic_spline() = default;

  std::vector<geometry_msgs::msg::PoseStamped>
  generate_path(const std::vector<double> &x,
                const std::vector<double> &y) const {
    std::vector<geometry_msgs::msg::PoseStamped> path_poses;

    // Ensure that we have at least two points to generate a path
    if (x.size() < 2 || y.size() < 2 || x.size() != y.size()) {
      RCLCPP_ERROR(rclcpp::get_logger("CubicSpline"),
                   "Invalid input size: x and y vectors must have at least two "
                   "points and be of the same size.");
      return path_poses; // Return an empty vector if the input is invalid
    }

    try {
      cubic_spline_interpolator spline_x(x, y);

      int num_points = 100;
      for (int i = 0; i < num_points; ++i) {
        double xi = x.front() + i * (x.back() - x.front()) / (num_points - 1);
        double yi = spline_x(xi);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = xi;
        pose.pose.position.y = yi;
        pose.pose.orientation.w = 1.0;

        path_poses.push_back(pose);
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("CubicSpline"),
                   "Exception occurred during spline interpolation: %s",
                   e.what());
      return path_poses; // Return an empty vector if there's an error
    }

    return path_poses; // Always return the vector, even if empty
  }

private:
  class cubic_spline_interpolator {
  public:
    cubic_spline_interpolator(const std::vector<double> &x,
                              const std::vector<double> &y) {
      if (x.size() != y.size()) {
        throw std::invalid_argument("x and y must have the same size");
      }

      int n = x.size();
      Eigen::VectorXd h(n - 1);
      for (int i = 0; i < n - 1; ++i) {
        h[i] = x[i + 1] - x[i];
      }

      Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
      Eigen::VectorXd b = Eigen::VectorXd::Zero(n);

      for (int i = 1; i < n - 1; ++i) {
        A(i, i - 1) = h[i - 1];
        A(i, i) = 2 * (h[i - 1] + h[i]);
        A(i, i + 1) = h[i];
        b[i] = 3 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1]);
      }

      A(0, 0) = 1;
      A(n - 1, n - 1) = 1;

      Eigen::VectorXd c = A.colPivHouseholderQr().solve(b);

      _a = y;
      _b.resize(n - 1);
      _c.resize(n - 1);
      _d.resize(n - 1);

      for (int i = 0; i < n - 1; ++i) {
        _c[i] = c[i];
        _b[i] = (y[i + 1] - y[i]) / h[i] - h[i] * (c[i + 1] + 2 * c[i]) / 3.0;
        _d[i] = (c[i + 1] - c[i]) / (3.0 * h[i]);
      }

      _x = x;
    }

    double operator()(double xi) const {
      if (xi < _x.front() || xi > _x.back()) {
        throw std::out_of_range("xi is out of range");
      }

      int i = findInterval(xi);
      double dx = xi - _x[i];
      return _a[i] + _b[i] * dx + _c[i] * dx * dx + _d[i] * dx * dx * dx;
    }

  private:
    std::vector<double> _x, _a, _b, _c, _d;

    int findInterval(double xi) const {
      auto it = std::lower_bound(_x.begin(), _x.end(), xi);
      return std::max(int(it - _x.begin()) - 1, 0);
    }
  };
};

#endif // CUBIC_SPLINE_HPP
