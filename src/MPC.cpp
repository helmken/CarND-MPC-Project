#include "FG_eval.h"
#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"


using namespace std;
using CppAD::AD;


//
// MPC class implementation.
//

MPC::MPC() 
{
}

MPC::~MPC() 
{
}


vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd fittedPolyCoeffs) 
{
    typedef CPPAD_TESTVECTOR(double) Dvector;

    const double x =      state[0];
    const double y =      state[1];
    const double psi =    state[2];
    const double v =      state[3];
    const double ctErr =  state[4];
    const double psiErr = state[5];

    // number of independent variables
    // numTimeSteps -> numTimeSteps - 1 actuations
    const size_t numIndepVars =     numTimeSteps * dimState             // numTimeSteps * 6 state variables
                                +   (numTimeSteps - 1) * dimActuator;   // (numTimeSteps - 1) * 2 control variables

    // Initial value of the independent variables.
    // Should be 0 except for the initial values.
    Dvector indepVars(numIndepVars);
    for (size_t i(0); i < numIndepVars; ++i)
    {
        indepVars[i] = 0.0;
    }

    // Set the initial variable values
    indepVars[startIdxX] =      x;
    indepVars[startIdxY] =      y;
    indepVars[startIdxPsi] =    psi;
    indepVars[startIdxV] =      v;
    indepVars[startIdxCTErr] =  ctErr;
    indepVars[startIdxPsiErr] = psiErr;

    cout << "numTimeSteps: " << numTimeSteps << ", numIndepVars: " << numIndepVars 
        << "\nindepVars: " << indepVars << "\n";

    // Lower and upper limits for state vector x0
    Dvector lowerBoundsOfVars(numIndepVars);
    Dvector upperBoundsOfVars(numIndepVars);

    // Set all non-actuators upper and lower limits
    // to the max negative and positive values.
    // setting limits for x, y, psi, v, ctErr, psiErr
    for (size_t i(0); i < startIdxDelta; ++i)
    {
        lowerBoundsOfVars[i] = -1.0e19;
        upperBoundsOfVars[i] = 1.0e19;
    }

    // The upper and lower limits of delta (steering angle) are set 
    // to -25 and 25 degrees (values in radians).
    // NOTE: Feel free to change this to something else.
    for (size_t i(startIdxDelta); i < startIdxA; ++i)
    {
        lowerBoundsOfVars[i] = -0.436332 * Lf;
        upperBoundsOfVars[i] = 0.436332 * Lf;
    }

    // Acceleration/deceleration upper and lower limits.
    // NOTE: Feel free to change this to something else.
    for (size_t i(startIdxA); i < numIndepVars; ++i)
    {
        lowerBoundsOfVars[i] = -1.0;
        upperBoundsOfVars[i] = 1.0;
    }

    // Number of constraints
    const size_t numConstraints(numTimeSteps * dimState);

    // Lower and upper limits for constraints
    // All of these should be 0 except the initial
    // state indices.
    Dvector lowerBoundsOfConstraints(numConstraints);
    Dvector upperBoundsOfConstraints(numConstraints);
    for (size_t i(0); i < numConstraints; ++i)
    {
        lowerBoundsOfConstraints[i] = 0;
        upperBoundsOfConstraints[i] = 0;
    }

    // constraints for initial state, otherwise the solver doesn't 
    // know where to start from
    lowerBoundsOfConstraints[startIdxX] =       x;
    lowerBoundsOfConstraints[startIdxY] =       y;
    lowerBoundsOfConstraints[startIdxPsi] =     psi;
    lowerBoundsOfConstraints[startIdxV] =       v;
    lowerBoundsOfConstraints[startIdxCTErr] =   ctErr;
    lowerBoundsOfConstraints[startIdxPsiErr] =  psiErr;

    upperBoundsOfConstraints[startIdxX] =       x;
    upperBoundsOfConstraints[startIdxY] =       y;
    upperBoundsOfConstraints[startIdxPsi] =     psi;
    upperBoundsOfConstraints[startIdxV] =       v;
    upperBoundsOfConstraints[startIdxCTErr] =   ctErr;
    upperBoundsOfConstraints[startIdxPsiErr] =  psiErr;

    cout << "numConstraints: " << numConstraints << "\n";
    cout << "lowerBoundsOfConstraints: " << lowerBoundsOfConstraints << "\n";
    cout << "upperBoundsOfConstraints: " << upperBoundsOfConstraints << "\n";

    // object that computes objective and constraints
    FG_eval fg_eval(fittedPolyCoeffs);

    // NOTE: You don't have to worry about these options
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
        indepVars, lowerBoundsOfVars, upperBoundsOfVars, 
        lowerBoundsOfConstraints, upperBoundsOfConstraints, 
        fg_eval, solution);

    // Check some of the solution values
    //bool ok = true;
    //ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;

    cout << "solution size: " << solution.x.size() << ", solution: " << solution.x << "\n";

    // TODO: Return the first actuator values. The variables can be accessed with
    // `solution.x[i]`.
    //
    // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
    // creates a 2 element double vector.

    vector<double> result;
    result.push_back(solution.x[startIdxDelta]);
    result.push_back(solution.x[startIdxA]);
    // delta and a are actually the important values

    // the values below are used to draw the green line, 
    // i.e. visualize where the car will go in the future
    for (size_t i(0); i < numTimeSteps - 1; ++i)
    {
        result.push_back(solution.x[startIdxX + i + 1]);
        result.push_back(solution.x[startIdxY + i + 1]);
    }

    return result;
}
