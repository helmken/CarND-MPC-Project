#include "FG_eval.h"
#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"


using namespace std;
using CppAD::AD;


//
// MPC class definition implementation.
//
MPC::MPC() 
{
}

MPC::~MPC() 
{
}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd fittedPolyCoeffs) 
{
    //cout << "state:" << state << "\n";
    //cout << "fittedPolyCoeffs:" << fittedPolyCoeffs << "\n";

    typedef CPPAD_TESTVECTOR(double) Dvector;

    const int stateDim(6);
    const int actuatorDim(2);

    // TODO: this is wrong, correct is below 
    const size_t numVars = N * stateDim * (N - 1) * actuatorDim;
    //const size_t numVars = N * stateDim + (N - 1) * actuatorDim;

    // Initial value of the independent variables.
    // should be 0 besides in initial state.
    Dvector vars(numVars);
    for (size_t i(0); i < numVars; ++i) 
    {
        vars[i] = 0;
    }

    Dvector vars_lowerbound(numVars);
    Dvector vars_upperbound(numVars);
    
    // TODO: Set lower and upper limits for variables.

    // Set all non-actuators upper and lower limits
    // to the max negative and positive values.
    // setting limits for x, y, psi, v, cte, psie
    for (size_t i(0); i < delta_startIdx; ++i)
    {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // The upper and lower limits of delta (steering angle) are set 
    // to -25 and 25 degrees (values in radians).
    // NOTE: Feel free to change this to something else.
    for (size_t i(delta_startIdx); i < a_startIdx; ++i)
    {
        vars_lowerbound[i] = -0.436332 * Lf;
        vars_upperbound[i] = 0.436332 * Lf;
    }

    // Acceleration/deceleration upper and lower limits.
    // NOTE: Feel free to change this to something else.
    for (size_t i(a_startIdx); i < numVars; ++i)
    {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    // TODO: Set the number of constraints
    const size_t numConstraints = N * stateDim;
    //cout << "numConstraints: " << numConstraints << "\n";
    //cout << "start indices: x=" << x_startIdx <<
    //    ", y=" << y_startIdx <<
    //    ", psi=" << psi_startIdx <<
    //    ", v=" << v_startIdx <<
    //    ", cte=" << cte_startIdx <<
    //    ", epsi=" << epsi_startIdx << 
    //    ", delta=" << delta_startIdx <<
    //    ", a=" << a_startIdx << "\n";

    // Lower and upper limits for the constraints
    // constraints should be 0 besides in initial state
    Dvector constraints_lowerbound(numConstraints);
    Dvector constraints_upperbound(numConstraints);
    for (size_t i(0); i < numConstraints; ++i)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    // constraints for initial state, otherwise the solver doesn't 
    // know where to start from
    constraints_lowerbound[x_startIdx] = state[0];
    constraints_upperbound[x_startIdx] = state[0];

    constraints_lowerbound[y_startIdx] = state[1];
    constraints_upperbound[y_startIdx] = state[1];

    constraints_lowerbound[psi_startIdx] = state[2];
    constraints_upperbound[psi_startIdx] = state[2];

    constraints_lowerbound[v_startIdx] = state[3];
    constraints_upperbound[v_startIdx] = state[3];

    constraints_lowerbound[cte_startIdx] = state[4];
    constraints_upperbound[cte_startIdx] = state[4];

    constraints_lowerbound[epsi_startIdx] = state[5];
    constraints_upperbound[epsi_startIdx] = state[5];

    // object that computes objective and constraints
    FG_eval fg_eval(fittedPolyCoeffs);

    //
    // NOTE: You don't have to worry about these options
    //
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    // TODO: if N is set to a larger value, max_cpu_time has to be increased
    options += "Numeric max_cpu_time          0.5\n"; 

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, 
        vars, vars_lowerbound, vars_upperbound, 
        constraints_lowerbound, constraints_upperbound, 
        fg_eval, solution);

    //bool ok = true;

    // Check some of the solution values
    //ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;

    // TODO: Return the first actuator values. The variables can be accessed with
    // `solution.x[i]`.
    //
    // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
    // creates a 2 element double vector.
    // return {};

    vector<double> result;
    result.push_back(solution.x[delta_startIdx]);
    result.push_back(solution.x[a_startIdx]);
    // delta and a are actually the important values

    // the values below are used to draw the green line, 
    // i.e. visualize where the car will go in the future
    for (size_t i(0); i < N - 1; ++i)
    {
        result.push_back(solution.x[x_startIdx + i + 1]);
        result.push_back(solution.x[y_startIdx + i + 1]);
    }

    cout << "solution:" << solution.x << "\n";

    return result;
}
