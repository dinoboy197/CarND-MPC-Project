#ifndef MPC_H
#define MPC_H

#include <memory>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "solution.h"

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

class MPC {
 public:

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  std::unique_ptr<Solution> solve(const Eigen::VectorXd state, const Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
