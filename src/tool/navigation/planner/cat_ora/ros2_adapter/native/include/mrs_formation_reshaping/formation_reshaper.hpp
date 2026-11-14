#ifndef __FORMATION_RESHAPER_H__
#define __FORMATION_RESHAPER_H__

#include <vector>
#include <numeric>
#include <optional>
#include <Eigen/Dense>
#include <utility>

namespace mrs_formation_reshaping
{

/**
 * @brief Class FormationReshaper
 */
class FormationReshaper {
public:
  /**
   * @brief constructor
   */
  FormationReshaper();

  /**
   * @brief constructor
   */
  FormationReshaper(double max_acc, double max_vel, double trajectory_dt);

  /**
   * @brief destructor
   */
  ~FormationReshaper(void);

  std::vector<Eigen::Vector3d> getMinimumTimeTrajectory(Eigen::Vector3d start, Eigen::Vector3d goal, double time_total, double ts, double am, double vm,
                                                        double dm);

  std::pair<std::vector<std::vector<Eigen::Vector3d>>, double> generateTrajectoriesMinTime(const std::vector<Eigen::Vector3d> &    starts,
                                                                                           const std::vector<Eigen::Vector3d> &    goals,
                                                                                           const std::vector<std::pair<int, int>> &f_mapping);

  std::pair<std::vector<std::pair<int, int>>, std::vector<double>> getCatoraAssignment(const std::vector<Eigen::Vector3d> &robot_positions,
                                                                                       const std::vector<Eigen::Vector3d> &formation_goals);

  std::pair<std::vector<std::pair<int, int>>, std::vector<double>> getLsapAssignment(const std::vector<Eigen::Vector3d> &robot_positions,
                                                                                     const std::vector<Eigen::Vector3d> &formation_goals);

  std::vector<std::vector<Eigen::Vector3d>> getReshapingTrajectoriesCatora(const std::vector<Eigen::Vector3d> &robot_positions,
                                                                           const std::vector<Eigen::Vector3d> &formation_goals);

  std::vector<std::vector<Eigen::Vector3d>> getReshapingTrajectoriesLsap(const std::vector<Eigen::Vector3d> &robot_positions,
                                                                         const std::vector<Eigen::Vector3d> &formation_goals);

  void setConstraintsAndDt(double max_acc, double max_vel, double trajectory_dt);

protected:
  double max_acc_       = 1.0;
  double max_vel_       = 1.0;
  double trajectory_dt_ = 0.2;

  int getIndexWithOne(const std::vector<int> &v);

  double getMinInitialDist(const std::vector<Eigen::Vector3d> &initial_positions, const std::vector<Eigen::Vector3d> &final_positions);

  std::vector<std::vector<int>> getDistanceMatrixEuclidean(const std::vector<Eigen::Vector3d> &start_points, const std::vector<Eigen::Vector3d> &goal_points);

  void getBoundedDistMatrix(std::vector<std::vector<int>> &distances, int limit_value, int max_value = 2e7);  // UNUSED

  std::vector<std::vector<Eigen::Vector3d>> getReshapingTrajectories(const std::vector<Eigen::Vector3d> &robot_positions,
                                                                     const std::vector<Eigen::Vector3d> &formation_goals, bool use_catora);
};

}  // namespace mrs_formation_reshaping

#endif
