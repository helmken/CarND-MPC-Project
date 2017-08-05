## MPC Project

The goals / steps of this project are the following:

An MPC is implemented so that the vehicle is able to follow the track with the given actuator latency of 0.1 s without leaving the track or hitting the curbs.

## Rubric Points
* an MPC should be implemented that can handle 100 ms actuator latency so that the vehicle doesn't hit the curbs or leaves the track
 * it has to be explained how latency was incorporated  
* the implemented model should be described in detail
 * state, actuators and update equations
* the chosen hyperparameter values for N and dt should be discussed, it should also explained which values were tried out 
* if the waypoints are preprocessed, the preprocessing has to be explained

---
### Writeup

### 1. MPC implementation
An MPC is implemented in the files `main.cpp`, `MPC.h`, `MPC.cpp` and `FG_eval.h`. 
The implementation is based on the given project code, the MPC Quizzes lesson and the Q&A video.

#### `main.cpp`
In `main.cpp` the values from the simulator are extracted and fed into the MPC.

First the future vehicle state is approximated based on the assumed actuator latency of 0.1 s:
```
// do some kind of prediction based on approximated
// yaw rate and acceleration
const double Lf = 2.67; // this is already defined in FT_eval.h
const double actuatorLatency = 0.1; // 0.1 seconds
px += v * cos(psi) * actuatorLatency;
py += v * sin(psi) * actuatorLatency;
psi -= v * steering * actuatorLatency / Lf; // -= to flip steering
v += throttle * actuatorLatency;
```
The approximation is not very accurate because it intermingles 
- the steering angle with yaw rate in the calculation of the yaw `psi` and
- the acceleration with the throttle value in the calculation of the velocity `v`.

This could be improved by calculating the yaw rate and acceleration from a history of previous states.

Next the given waypoints are preprocessed as suggested in the Q&A video. They are rotated by the yaw of the vehicle to ease the fitting of a polynomial:
```
vector<double> waypointsTransX;
vector<double> waypointsTransY;

// shift car reference angle to 90 degrees
// actually the rotation is not with 90 degrees, but
// instead its rotating around the yaw of the car
// the reference coordinate system has then zero degrees
for (size_t i(0); i < waypointsX.size(); ++i)
{
    // shift
    const double shift_x = waypointsX[i] - px;
    const double shift_y = waypointsY[i] - py;

    // rotate: minus yaw
    const double rotX = (shift_x * cos(0 - psi) - shift_y * sin(0 - psi));
    const double rotY = (shift_x * sin(0 - psi) + shift_y * cos(0 - psi));
    waypointsTransX.push_back(rotX);
    waypointsTransY.push_back(rotY);
}
```

These rotated waypoints are then used to fit a 3rd order polynomial:
```
double* ptrx = &waypointsTransX[0];
const Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);

double* ptry = &waypointsTransY[0];
const Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);

const Eigen::VectorXd fittedPolyCoeffs = polyfit(ptsx_transform, ptsy_transform, 3);
```

From the coefficients of the polynomial the cross track error and the yaw error is calculated and inserted into a state vector that is then used to find an optimal solution:
```
// calculate cte and epsi
const double ctErr = polyeval(fittedPolyCoeffs, 0);

//double psiErr = psi - atan(coeffs[1] + 2 * px * coeffs[2] + 3 * coeffs[3] * pow(px, 2));
const double psiErr = -atan(fittedPolyCoeffs[1]); // simplified version of above line (because of previous shift and rotation)

Eigen::VectorXd state(6);
state << 0, 0, 0, v, ctErr, psiErr;
```

`MPC::Solve(...)` returns the next controller values for steering and throttle and additionally the coordinates of the optimal path that are used as a visual debugging aid.

#### `MPC.cpp`
`vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd fittedPolyCoeffs)` finds an optimal solution for the given `state` and the polynomial coefficients of the waypoints.

First the number of independent variables is calculated:
```
    // number of independent variables
    // numTimeSteps -> numTimeSteps - 1 actuations
    const size_t numIndepVars =     numTimeSteps * dimState             // numTimeSteps * 6 state variables
                                +   (numTimeSteps - 1) * dimActuator;   // (numTimeSteps - 1) * 2 control variables
```
The number is determined by the hyperparameter for the number of time steps, the dimension of the state and the dimension of controller input. It is used to create arrays that are fed into the solver:
- `Dvector indepVars(numIndepVars)` is filled with state and error values at the respective indices `startIdxX, startIdxY, startIdxPsi, startIdxV, startIdxCTErr, startIdxPsiErr` and zeroes otherwise.
- `Dvector lowerBoundsOfVars(numIndepVars), Dvector upperBoundsOfVars(numIndepVars)` are used to provide upper and lower bounds of state and error variables to the solver. These are filled with large negative and positive values for the part of the state and with maximum values for steering (+- `0.436332 * Lf`) and throttle (+- 1.0).

Additionally two arrays are created to provide the constraints to the solver: `Dvector lowerBoundsOfConstraints(numConstraints)` and `Dvector upperBoundsOfConstraints(numConstraints)`. These are filled with the values of the state at the respective indices `startIdxX, startIdxY, startIdxPsi, startIdxV, startIdxCTErr, startIdxPsiErr` and otherwise zeroes. 

These arrays are then fed into the solver like this:
```
CppAD::ipopt::solve<Dvector, FG_eval>(
    options, 
    indepVars, lowerBoundsOfVars, upperBoundsOfVars, 
    lowerBoundsOfConstraints, upperBoundsOfConstraints, 
    fg_eval, solution);
```

The result is sent back to the calling site in `main()` and then sent from there to the simulator.

#### `FG_eval.h`
The functor `FG_eval` is used to calculate the costs for the optimization.
In `operator()(ADvector& costAndConstraints, const ADvector& stateAndActuators)` the cost of a state is stored in in the first element of `costAndConstraints`. I used the suggested parameters from the Q&A video:
```
// The part of the cost based on the reference state.
for (size_t i(0); i < numTimeSteps; ++i)
{
    costAndConstraints[0] += 2000 * CppAD::pow(stateAndActuators[startIdxCTErr + i] - refCTErr, 2);
    costAndConstraints[0] += 2000 * CppAD::pow(stateAndActuators[startIdxPsiErr + i] - refPsiErr, 2);
    costAndConstraints[0] += CppAD::pow(stateAndActuators[startIdxV + i] - refVel, 2);
}
```
The penalty factor 2000 for cross track error and yaw error is relatively large, meaning that the vehicle should stay on the track at all means. The velocity error is just added as it is, meaning that a high velocity is not too important.  

```
// Minimize the use of actuators.
for (size_t i(0); i < numTimeSteps - 1; ++i)
{
    costAndConstraints[0] += 5 * CppAD::pow(stateAndActuators[startIdxDelta + i], 2);
    costAndConstraints[0] += 5 * CppAD::pow(stateAndActuators[startIdxA + i], 2);
}
```
Here both penalty values of 5 are relatively low meaning that extreme controls like full throttle or maximum steering are allowed as necessary.

```
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
```
Here the diffence between consecutive actuations is penalized: The penalty factor of 200 for steering changes is relatively high to throttle changes which is 10, meaning that sudden changes of direction should be avoided, while strong acceleration and braking are not too bad.

Afterwards the derivatives are calculated and stored in `costAndConstraints` for all time steps:
```
costAndConstraints[startIdxX + i + 2] =         x1 -        (x0 +   v0 * CppAD::cos(psi0) * dt);
costAndConstraints[startIdxY + i + 2] =         y1 -        (y0 +   v0 * CppAD::sin(psi0) * dt);
costAndConstraints[startIdxPsi + i + 2] =       psi1 -      (psi0 + v0 * delta0 / Lf * dt);
costAndConstraints[startIdxV + i + 2] =         v1 -        (v0 +   a0 * dt);

costAndConstraints[startIdxCTErr + i + 2] =     ctErr1 -    ((f0 - y0) + (v0 * CppAD::sin(psiErr0) * dt));
costAndConstraints[startIdxPsiErr + i + 2] =    psiErr1 -   ((psi0 - psiDes0) + v0 * delta0 / Lf * dt);
```




### 2. Model description
The state update is based on the equations
```
// do some kind of prediction based on approximated
// yaw rate and acceleration
const double Lf = 2.67; // this is already defined in FT_eval.h
const double actuatorLatency = 0.1; // 0.1 seconds
px += v * cos(psi) * actuatorLatency;
py += v * sin(psi) * actuatorLatency;
psi -= v * steering * actuatorLatency / Lf; // -= to flip steering
v += throttle * actuatorLatency;
```
(see `main.cpp`). I first tried to incorporate the actuator latency in `FG_eval.h` as described below, by adding the latency to dt, but this didnt work out as intented.

That is then incorporated in the solver constraints like this:
```
costAndConstraints[startIdxX + i + 2] =         x1 -        (x0 +   v0 * CppAD::cos(psi0) * dt);
            costAndConstraints[startIdxY + i + 2] =         y1 -        (y0 +   v0 * CppAD::sin(psi0) * dt);
            costAndConstraints[startIdxPsi + i + 2] =       psi1 -      (psi0 + v0 * delta0 / Lf * dt);
            costAndConstraints[startIdxV + i + 2] =         v1 -        (v0 +   a0 * dt);
            
            costAndConstraints[startIdxCTErr + i + 2] =     ctErr1 -    ((f0 - y0) + (v0 * CppAD::sin(psiErr0) * dt));
            costAndConstraints[startIdxPsiErr + i + 2] =    psiErr1 -   ((psi0 - psiDes0) + v0 * delta0 / Lf * dt);
```
(see file `FG_eval.h`).

The state is made up of 
- x and y coordinate 
- yaw
- velocity
- cross track error
- deviation of yaw

The actuators are the steering angle and throttle. The steering angle needed a sign flip, because the model uses positive y values for leftward while the simulator uses negative values for steering left.

### 3. Chosen Hyperparameters
The number of time steps N and the time difference dt determine, as taught in the lesson, the prediction horizon. For a moving vehicle it does not make sense to plan the motion too far in the future, because the environment could change significantly, so intuitively a horizon of a few seconds should be sufficient. 
The time difference dt determines the resolution of the prediction:
- if its too coarse, the prediction would be inaccurate. In the MPC Quizzes lesson I noticed that if I use a larger value for dt, e.g. 0.5, the resultant graphs look like jigsaws
- if its too fine, computation time is wasted that could be used for more meaningful tasks

For the given velocity range I found that 0.1 seconds as time difference should be good enough. Based on dt I thought that 1 s for prediction would be the minimum, so I chose 10 for the number of time steps. I found no improvement with larger values e.g. N=20. From the visual comparison of the green and yellow line I found that the green line (planned motion) pretty much covered the significant part of the yellow line (given waypoints) for most of the time with a value of 10.

I first used the given penalty parameters for the cost function and tried to fiddle around with them, but found no significant improvement - so I left the original values.

### 4. Waypoint preprocessing
I used the method presented in the Q&A video to rotate the waypoints around the yaw, to simplify fitting the polynomial:
```
vector<double> waypointsTransX;
vector<double> waypointsTransY;

// shift car reference angle to 90 degrees
// actually the rotation is not with 90 degrees, but
// instead its rotating around the yaw of the car
// the reference coordinate system has then zero degrees
for (size_t i(0); i < waypointsX.size(); ++i)
{
    // shift
    const double shift_x = waypointsX[i] - px;
    const double shift_y = waypointsY[i] - py;

    // rotate: minus yaw
    const double rotX = (shift_x * cos(0 - psi) - shift_y * sin(0 - psi));
    const double rotY = (shift_x * sin(0 - psi) + shift_y * cos(0 - psi));
    waypointsTransX.push_back(rotX);
    waypointsTransY.push_back(rotY);
}
```

