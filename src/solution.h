#ifndef SOLUTION_H
#define SOLUTION_H

#include <memory>

struct Solution {
  const std::vector<double> path_x;
  const std::vector<double> path_y;
  const double steering_actuation;
  const double throttle_actuation;

  Solution(const std::vector<double> path_x, const std::vector<double> path_y,
           const double steering_actuation, const double throttle_actuation) :
    path_x(path_x), path_y(path_y), steering_actuation(steering_actuation), throttle_actuation(throttle_actuation) {}
};

#endif /* SOLUTION_H */
