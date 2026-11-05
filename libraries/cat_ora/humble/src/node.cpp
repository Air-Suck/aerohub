#include <cmath>
#include <memory>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include "mrs_formation_reshaping/formation_reshaper.hpp"
#include "mrs_formation_reshaping/srv/srv_get_reshaping_trajectories.hpp"

using Service = mrs_formation_reshaping::srv::SrvGetReshapingTrajectories;

class CatOraNode : public rclcpp::Node {
public:
  CatOraNode() : Node("mrs_formation_reshaping") {
    service_ = create_service<Service>(
      "~/get_reshaping_trajectories",
      [this](const Service::Request::SharedPtr request, Service::Response::SharedPtr response) {
        try {
          if (request->publish_visualization)
            throw std::invalid_argument("visualization is not supported by this service port");
          if (request->initial_configurations.empty() ||
              request->initial_configurations.size() != request->goal_configurations.size())
            throw std::invalid_argument("configurations must be nonempty and have equal sizes");
          for (double value : {request->max_acceleration, request->max_velocity, request->trajectory_dt})
            if (!std::isfinite(value) || value < 1e-3)
              throw std::invalid_argument("constraints must be finite and at least 0.001");
          auto convert = [](const auto &points) {
            std::vector<Eigen::Vector3d> result;
            for (const auto &p : points) {
              if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                throw std::invalid_argument("coordinates must be finite");
              result.emplace_back(p.x, p.y, p.z);
            }
            return result;
          };
          const auto starts = convert(request->initial_configurations);
          const auto goals = convert(request->goal_configurations);
          // The upstream callback reverses these two arguments; the core signature is (acc, vel, dt).
          planner_.setConstraintsAndDt(request->max_acceleration, request->max_velocity, request->trajectory_dt);
          const auto trajectories = planner_.getReshapingTrajectoriesCatora(starts, goals);
          if (trajectories.size() != starts.size())
            throw std::runtime_error("CAT-ORA returned an unexpected trajectory count");
          response->trajectories.resize(trajectories.size());
          for (size_t i = 0; i < trajectories.size(); ++i) {
            for (const auto &v : trajectories[i]) {
              geometry_msgs::msg::Point p;
              p.x = v.x(); p.y = v.y(); p.z = v.z();
              response->trajectories[i].waypoints.push_back(p);
            }
          }
          response->success = true;
          response->message = "success";
        } catch (const std::exception &error) {
          response->success = false;
          response->trajectories.clear();
          response->message = error.what();
        }
      });
  }
private:
  mrs_formation_reshaping::FormationReshaper planner_;
  rclcpp::Service<Service>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CatOraNode>());
  rclcpp::shutdown();
}
