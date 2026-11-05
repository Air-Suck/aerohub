// Compile this same harness against both original and ported optimizer libraries.
#include <optimizer/poly_traj_optimizer.h>
#include <chrono>
#include <fstream>
#include <iomanip>
#ifdef SWARM_HUMBLE
#include <rcl/time.h>
#endif

int main(int argc, char **argv) {
#ifdef SWARM_HUMBLE
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("optimizer_benchmark");
  swarm_ros::node = nh;
  struct Cleanup {
    rclcpp::Node::SharedPtr &nh;
    ~Cleanup() { swarm_ros::node.reset(); nh.reset(); rclcpp::shutdown(); }
  } cleanup{nh};
  if (rcl_enable_ros_time_override(nh->get_clock()->get_clock_handle()) != RCL_RET_OK ||
      rcl_set_ros_time_override(nh->get_clock()->get_clock_handle(), 100000000000LL) != RCL_RET_OK)
    return 2;
#else
  ros::init(argc, argv, "optimizer_benchmark");
  ros::NodeHandle nh("~");
  ros::Time::setNow(ros::Time(100));
#endif
  if (argc < 2) return 2;
  std::ofstream out(argv[1]);
  out << std::setprecision(17) << "[";
  auto map = std::make_shared<GridMap>();
  map->initMap(nh);
  ego_planner::PolyTrajOptimizer optimizer;
  optimizer.setParam(nh);
  optimizer.setEnvironment(map);
  const std::vector<Eigen::Vector3d> offsets = {
    {0,0,0}, {3.4642,-2,0}, {0,-4,0}, {-3.4642,-2,0},
    {-3.4642,2,0}, {0,4,0}, {3.4642,2,0}};
  bool first = true;
  for (int scene = 0; scene < 4; ++scene) {
    ego_planner::SwarmTrajData swarm(7);
    Eigen::VectorXd durations(3);
    durations << 1.4, 1.6, 1.8;
    std::vector<Eigen::Matrix3d> heads(7), tails(7);
    std::vector<Eigen::MatrixXd> inner(7);
    for (int id = 0; id < 7; ++id) {
      heads[id].setZero(); tails[id].setZero();
      heads[id].col(0) = offsets[id] + Eigen::Vector3d(-5,0,1.5);
      tails[id].col(0) = heads[id].col(0) + Eigen::Vector3d(3+scene, scene*0.4, 0.1*scene);
      if (scene == 3) tails[id](1,0) += 0.1*id;
      inner[id].resize(3,2);
      inner[id].col(0) = heads[id].col(0)*2/3 + tails[id].col(0)/3;
      inner[id].col(1) = heads[id].col(0)/3 + tails[id].col(0)*2/3;
      poly_traj::MinJerkOpt reference;
      reference.reset(heads[id], tails[id], 3);
      reference.generate(inner[id], durations);
      swarm[id].drone_id = id;
      swarm[id].start_time = 100.0;
      swarm[id].duration = reference.getTraj().getTotalDuration();
      swarm[id].traj = reference.getTraj();
    }
    optimizer.setSwarmTrajs(&swarm);
    for (int formation = 0; formation < 2; ++formation) {
      for (int id = 0; id < 7; ++id) {
        optimizer.setDroneId(id);
        for (int repeat = -3; repeat < 15; ++repeat) {
          poly_traj::MinJerkOpt initial;
          initial.reset(heads[id], tails[id], 3);
          initial.generate(inner[id], durations);
          optimizer.setControlPoints(initial.getInitConstrainPoints(optimizer.get_cps_num_prePiece_()));
          Eigen::MatrixXd points;
          auto begin = std::chrono::steady_clock::now();
          bool success = optimizer.OptimizeTrajectory_lbfgs(
            heads[id], tails[id], inner[id], durations, points, formation);
          double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-begin).count();
          if (repeat < 0) continue;
          auto trajectory = optimizer.getMinJerkOptPtr()->getTraj();
          if (!first) out << ",";
          first = false;
          out << "{\"scene\":" << scene << ",\"formation\":" << formation
              << ",\"drone\":" << id << ",\"repeat\":" << repeat
              << ",\"success\":" << (success ? "true" : "false")
              << ",\"ms\":" << ms << ",\"duration\":" << trajectory.getTotalDuration()
              << ",\"samples\":[";
          for (int k = 0; k <= 50; ++k) {
            if (k) out << ",";
            auto p = trajectory.getPos(trajectory.getTotalDuration()*k/50);
            out << "[" << p.x() << "," << p.y() << "," << p.z() << "]";
          }
          out << "]}";
        }
      }
    }
  }
  out << "]\n";
  return out.good() ? 0 : 3;
}
