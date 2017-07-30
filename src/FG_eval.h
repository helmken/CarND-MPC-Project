#ifndef FG_EVAL_H
#define FG_EVAL_H


#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"


using CppAD::AD;


// Prediction horizon T = N * dt
// TODO: Set N and dt
size_t N = 10;
double dt = 0.1;

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

double ref_cte = 0;
double ref_epsi = 0;
//double ref_v = 100; // TODO: velocity 100 is fast, maybe reduce it
double ref_v = 10; // TODO: velocity 100 is fast, maybe reduce it

// state vector has N values for each dimension
const size_t x_startIdx = 0;
const size_t y_startIdx = x_startIdx + N;
const size_t psi_startIdx = y_startIdx + N;
const size_t v_startIdx = psi_startIdx + N;
const size_t cte_startIdx = v_startIdx + N;
const size_t epsi_startIdx = cte_startIdx + N;

// control vector has N - 1 values for each dimension
size_t delta_startIdx = epsi_startIdx + N;
size_t a_startIdx = delta_startIdx + N - 1;

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
        for (size_t i(0); i < N; ++i)
        {
            costAndConstraints[0] += 2000 * CppAD::pow(stateAndActuators[cte_startIdx + i] - ref_cte, 2);
            costAndConstraints[0] += 2000 * CppAD::pow(stateAndActuators[epsi_startIdx + i] - ref_epsi, 2);
            costAndConstraints[0] += CppAD::pow(stateAndActuators[v_startIdx + i] - ref_v, 2);
        }

		// Minimize the use of actuators.
        for (size_t i(0); i < N - 1; ++i)
        {
            costAndConstraints[0] += 5 * CppAD::pow(stateAndActuators[delta_startIdx + i], 2);
            costAndConstraints[0] += 5 * CppAD::pow(stateAndActuators[a_startIdx + i], 2);
        }

        // Minimize the value gap between sequential actuations.
		for (size_t i(0); i < N - 2; ++i)
        {
            costAndConstraints[0] += 200 * CppAD::pow(
                  stateAndActuators[delta_startIdx + i + 1]
                - stateAndActuators[delta_startIdx + i], 2);
            costAndConstraints[0] += 10 * CppAD::pow(
                  stateAndActuators[a_startIdx + i + 1]
                - stateAndActuators[a_startIdx + i], 2);
        }

        // setup model constraints

        // We add 1 to each of the starting indices due to cost being located at
        // index 0 of `costAndConstraints`.
        // This bumps up the position of all the other values.

        // initial constraints
        costAndConstraints[1 + x_startIdx] =    stateAndActuators[x_startIdx];
        costAndConstraints[1 + y_startIdx] =    stateAndActuators[y_startIdx];
        costAndConstraints[1 + psi_startIdx] =  stateAndActuators[psi_startIdx];
        costAndConstraints[1 + v_startIdx] =    stateAndActuators[v_startIdx];
        costAndConstraints[1 + cte_startIdx] =  stateAndActuators[cte_startIdx];
        costAndConstraints[1 + epsi_startIdx] = stateAndActuators[epsi_startIdx];

        // The rest of the constraints
        for (size_t i(0); i < N - 1; ++i)
        {
            // The state at time t+1.
            AD<double> x1 = stateAndActuators[x_startIdx + i + 1];
            AD<double> y1 = stateAndActuators[y_startIdx + i + 1];
            AD<double> psi1 = stateAndActuators[psi_startIdx + i + 1];
            AD<double> v1 = stateAndActuators[v_startIdx + i + 1];
            AD<double> cte1 = stateAndActuators[cte_startIdx + i + 1];
            AD<double> epsi1 = stateAndActuators[epsi_startIdx + i + 1];

            // The state at time t.
            AD<double> x0 = stateAndActuators[x_startIdx + i];
            AD<double> y0 = stateAndActuators[y_startIdx + i];
            AD<double> psi0 = stateAndActuators[psi_startIdx + i];
            AD<double> v0 = stateAndActuators[v_startIdx + i];
            AD<double> cte0 = stateAndActuators[cte_startIdx + i];
            AD<double> epsi0 = stateAndActuators[epsi_startIdx + i];

            AD<double> delta0 = stateAndActuators[delta_startIdx + i];
            AD<double> a0 = stateAndActuators[a_startIdx + i];

            // calculate value of polynomial
            AD<double> f0 =
                m_fittedPolyCoeffs[0] +
                m_fittedPolyCoeffs[1] * x0 +
                m_fittedPolyCoeffs[2] * x0 * x0 +
                m_fittedPolyCoeffs[3] * x0 * x0 * x0;
            AD<double> psides0 = CppAD::atan(
                3 * m_fittedPolyCoeffs[3] * x0 * x0 +
                2 * m_fittedPolyCoeffs[2] * x0 +
                m_fittedPolyCoeffs[1]);

            // TODO: Setup the rest of the model constraints
            // Recall the equations for the model:
            // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
            // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
            // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
            // v_[t+1] = v[t] + a[t] * dt
            // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
            // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

            // the idea here is to constraint this value to be 0
            // TODO: setup the rest of the model constraints
            // scheme: (value_at_t_+_1) - (value_at_t + rate_of_change)
            // it is inforced that the difference is zero, otherwise the solver would do weird stuff
            // CppAD calculates gradients

            costAndConstraints[x_startIdx + i + 2] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            costAndConstraints[y_startIdx + i + 2] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            costAndConstraints[psi_startIdx + i + 2] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
            costAndConstraints[v_startIdx + i + 2] = v1 - (v0 + a0 * dt);
            costAndConstraints[cte_startIdx + i + 2] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            costAndConstraints[epsi_startIdx + i + 2] = ((psi0 - psides0) - v0 * delta0 / Lf * dt);
        }
    }
};

#endif //FG_EVAL_H