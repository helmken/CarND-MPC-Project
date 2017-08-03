#ifndef FG_EVAL_H
#define FG_EVAL_H


#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"


using CppAD::AD;


// Prediction horizon T = numTimeSteps * dt
const size_t numTimeSteps = 10;
const double dt = 0.1;

const size_t dimState(6);
const size_t dimActuator(2);

const double refCTErr = 0;
const double refPsiErr = 0;
//double refVel(100); // TODO: velocity 100 is fast, maybe reduce it
const double refVel(40);

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in
// the simulator around in a circle with a constant steering angle and
// velocity on a flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG (Center of Gravity?) that has a
// similar radius.
const double Lf = 2.67;

// The solver takes all the state variables and actuator variables in a
// singular vector. Thus, we should to establish when one variable starts and
// another ends to make our lifes easier.
const size_t startIdxX =        0;
const size_t startIdxY =        startIdxX + numTimeSteps;
const size_t startIdxPsi =      startIdxY + numTimeSteps;
const size_t startIdxV =        startIdxPsi + numTimeSteps;
const size_t startIdxCTErr =    startIdxV + numTimeSteps;
const size_t startIdxPsiErr =   startIdxCTErr + numTimeSteps;

// control vector has N - 1 values for each dimension
size_t startIdxDelta =          startIdxPsiErr + numTimeSteps;
size_t startIdxA =              startIdxDelta + numTimeSteps - 1;

class FG_eval
{
public:
    // Fitted polynomial coefficients
    Eigen::VectorXd m_fittedPolyCoeffs;

    FG_eval(Eigen::VectorXd coeffs)
    {
        m_fittedPolyCoeffs = coeffs;
    }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector& costAndConstraints, const ADvector& stateAndActuators)
    {
        // The cost is stored is the first element of `costAndConstraints`.
        // Any additions to the cost should be added to `costAndConstraints[0]`.
        costAndConstraints[0] = 0;

        // Reference State Cost
        // TODO: Define the cost related the reference state and
        // any anything you think may be beneficial.

        // The part of the cost based on the reference state.
        for (size_t i(0); i < numTimeSteps; ++i)
        {
            costAndConstraints[0] += 2000 * CppAD::pow(stateAndActuators[startIdxCTErr + i] - refCTErr, 2);
            costAndConstraints[0] += 2000 * CppAD::pow(stateAndActuators[startIdxPsiErr + i] - refPsiErr, 2);
            costAndConstraints[0] += CppAD::pow(stateAndActuators[startIdxV + i] - refVel, 2);
        }

		// Minimize the use of actuators.
        for (size_t i(0); i < numTimeSteps - 1; ++i)
        {
            costAndConstraints[0] += 5 * CppAD::pow(stateAndActuators[startIdxDelta + i], 2);
            costAndConstraints[0] += 5 * CppAD::pow(stateAndActuators[startIdxA + i], 2);
        }

        // Minimize the value gap between sequential actuations.
		for (size_t i(0); i < numTimeSteps - 2; ++i)
        {
            costAndConstraints[0] += 200 * CppAD::pow(
                  stateAndActuators[startIdxDelta + i + 1]
                - stateAndActuators[startIdxDelta + i], 2);
            costAndConstraints[0] += 10 * CppAD::pow(
                  stateAndActuators[startIdxA + i + 1]
                - stateAndActuators[startIdxA + i], 2);
        }

        // setup model constraints

        // We add 1 to each of the starting indices due to cost being located at
        // index 0 of `costAndConstraints`.
        // This bumps up the position of all the other values.

        // initial constraints
        costAndConstraints[1 + startIdxX] =         stateAndActuators[startIdxX];
        costAndConstraints[1 + startIdxY] =         stateAndActuators[startIdxY];
        costAndConstraints[1 + startIdxPsi] =       stateAndActuators[startIdxPsi];
        costAndConstraints[1 + startIdxV] =         stateAndActuators[startIdxV];
        costAndConstraints[1 + startIdxCTErr] =     stateAndActuators[startIdxCTErr];
        costAndConstraints[1 + startIdxPsiErr] =    stateAndActuators[startIdxPsiErr];

        // The rest of the constraints
        for (size_t i(0); i < numTimeSteps - 1; ++i)
        {
            // The state at time t+1.
            AD<double> x1 =         stateAndActuators[startIdxX + i + 1];
            AD<double> y1 =         stateAndActuators[startIdxY + i + 1];
            AD<double> psi1 =       stateAndActuators[startIdxPsi + i + 1];
            AD<double> v1 =         stateAndActuators[startIdxV + i + 1];
            AD<double> ctErr1 =     stateAndActuators[startIdxCTErr + i + 1];
            AD<double> psiErr1 =    stateAndActuators[startIdxPsiErr + i + 1];

            // The state at time t.
            AD<double> x0 =         stateAndActuators[startIdxX + i];
            AD<double> y0 =         stateAndActuators[startIdxY + i];
            AD<double> psi0 =       stateAndActuators[startIdxPsi + i];
            AD<double> v0 =         stateAndActuators[startIdxV + i];
            AD<double> ctErr0 =     stateAndActuators[startIdxCTErr + i];
            AD<double> psiErr0 =    stateAndActuators[startIdxPsiErr + i];

            // Only consider the actuation at time t.
            AD<double> delta0 =     stateAndActuators[startIdxDelta + i];
            AD<double> a0 =         stateAndActuators[startIdxA + i];

            // calculate value of polynomial
            AD<double> f0 =
                m_fittedPolyCoeffs[0] +
                m_fittedPolyCoeffs[1] * x0 +
                m_fittedPolyCoeffs[2] * x0 * x0 +
                m_fittedPolyCoeffs[3] * x0 * x0 * x0;
            AD<double> psiDes0 = CppAD::atan(
                3 * m_fittedPolyCoeffs[3] * x0 * x0 +
                2 * m_fittedPolyCoeffs[2] * x0 +
                m_fittedPolyCoeffs[1]);

            // NOTE: The use of `AD<double>` and use of `CppAD`!
            // This is also CppAD can compute derivatives and pass
            // these to the solver.

            // TODO: Setup the rest of the model constraints
            // Recall the equations for the model:
            // x_[t+1] =    x[t] +      v[t] * cos(psi[t]) * dt
            // y_[t+1] =    y[t] +      v[t] * sin(psi[t]) * dt
            // psi_[t+1] =  psi[t] +    v[t] / Lf * delta[t] * dt
            // v_[t+1] =    v[t] +      a[t] * dt
            // cte[t+1] =   f(x[t]) -   y[t] + v[t] * sin(epsi[t]) * dt
            // epsi[t+1] =  psi[t] -    psides[t] + v[t] * delta[t] / Lf * dt

            // the idea here is to constraint this value to be 0
            // TODO: setup the rest of the model constraints
            // scheme: (value_at_t_+_1) - (value_at_t + rate_of_change)
            // it is inforced that the difference is zero, otherwise the solver would do weird stuff
            // CppAD calculates gradients

            costAndConstraints[startIdxX + i + 2] =         x1 -        (x0 +   v0 * CppAD::cos(psi0) * dt);
            costAndConstraints[startIdxY + i + 2] =         y1 -        (y0 +   v0 * CppAD::sin(psi0) * dt);
            costAndConstraints[startIdxPsi + i + 2] =       psi1 -      (psi0 + v0 * delta0 / Lf * dt);
            costAndConstraints[startIdxV + i + 2] =         v1 -        (v0 +   a0 * dt);
            
            costAndConstraints[startIdxCTErr + i + 2] =     ctErr1 -    ((f0 - y0) + (v0 * CppAD::sin(psiErr0) * dt));
            costAndConstraints[startIdxPsiErr + i + 2] =    psiErr1 -   ((psi0 - psiDes0) + v0 * delta0 / Lf * dt);
        }
    }
};

#endif // FG_EVAL_H