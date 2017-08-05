#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

class MPC 
{
public:
    MPC();

    virtual ~MPC();

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuations.
    std::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

private:
    double m_prevDelta;
    double m_prevA;
    bool m_initialized;
};

#endif /* MPC_H */
