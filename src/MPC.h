#ifndef MPC_H
#define MPC_H

#include <memory>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "solution.h"

class MPC {
 public:
  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  std::unique_ptr<Solution> solve(const Eigen::VectorXd state, const Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
