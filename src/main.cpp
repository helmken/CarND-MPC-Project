#include <cmath>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"


using namespace std;


// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
double deg2rad(double x) 
{ 
    return x * M_PI / 180;
}

double rad2deg(double x) 
{ 
    return x * 180 / M_PI;
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) 
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != string::npos) 
    {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) 
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) 
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Fit a polynomial, adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(
    Eigen::VectorXd xvals, 
    Eigen::VectorXd yvals,
    int order) 
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);

    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) 
    {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) 
    {
        for (int i = 0; i < order; i++) 
        {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    
    return result;
}

int main() 
{
    uWS::Hub uWsHub;

    // MPC is initialized here!
    MPC mpc;

    uWsHub.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
        uWS::OpCode opCode) 
    {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') 
        {
            string s = hasData(sdata);
            if (s != "") 
            {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") 
                {
                    // j[1] is the data JSON object
                    vector<double> waypointsX = j[1]["ptsx"];
                    vector<double> waypointsY = j[1]["ptsy"];
                    
                    //cout << "waypoints: ";
                    //for (size_t i(0); i < waypointsX.size(); ++i)
                    //{
                    //    cout << "(" << waypointsX[i] << ", " << waypointsY[i] << "), ";
                    //}
                    //cout << "\n";

                    const double px = j[1]["x"];
                    const double py = j[1]["y"];
                    const double psi = j[1]["psi"];
                    const double v = j[1]["speed"];

                    // TODO: unused?!? 
                    //double steer_value = j[1]["steering_angle"];

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

                    double* ptrx = &waypointsTransX[0];
                    const Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);

                    double* ptry = &waypointsTransY[0];
                    const Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);

                    const Eigen::VectorXd fittedPolyCoeffs = polyfit(ptsx_transform, ptsy_transform, 3);

                    // calculate cte and epsi
                    const double ctErr = polyeval(fittedPolyCoeffs, 0);

                    //double psiErr = psi - atan(coeffs[1] + 2 * px * coeffs[2] + 3 * coeffs[3] * pow(px, 2));
                    const double psiErr = -atan(fittedPolyCoeffs[1]); // simplified version of above line (because of previous shift and rotation)

                    
                    // throttle is not acceleration, but somehow an estimator for the acceleration
                    // TODO: unused?!? 
                    //double throttle_value = j[1]["throttle"]; 

                    Eigen::VectorXd state(6);
                    state << 0, 0, 0, v, ctErr, psiErr;

                    // TODO: Calculate steering angle and throttle using MPC.
                    // Both are in between [-1, 1].

                    // the coefficients are the path that the vehicle wants to follow
                    const vector<double> vars = mpc.Solve(state, fittedPolyCoeffs);

                    // used for visual debugging: display the line that the
                    // vehicle is trying to follow as a yellow line
                    vector<double> yellowLineX;
                    vector<double> yellowLineY;

                    const double stepSizeX = 2.5; // distance in x axis for which the polynomial is evaluated
                    const int numSteps = 25; // 25 points should be displayed
                    for (int i(1); i < numSteps; ++i)
                    {
                        yellowLineX.push_back(stepSizeX * i);
                        yellowLineY.push_back(polyeval(fittedPolyCoeffs, stepSizeX * i));
                    }

                    // this will be the green line
                    vector<double> greenLineX;
                    vector<double> greenLineY;
                    for (size_t i(2); i < vars.size(); ++i)
                    {
                        if (i % 2 == 0)
                        {
                            greenLineX.push_back(vars[i]);
                        }
                        else
                        {
                            greenLineY.push_back(vars[i]);
                        }
                    }
                    //cout << "number of green line points: " << greenLineX.size() << "\n";

                    double Lf = 2.67; // this is already defined in MPC.cpp

                    json msgJson;

                    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    //msgJson["steering_angle"] = steer_value;
                    //msgJson["throttle"] = throttle_value;
                    // these are the actually relevant control values that are send to simulator
                    msgJson["steering_angle"] = - vars[0] / (deg2rad(25) * Lf);
                    msgJson["throttle"] = vars[1];

                    //Display the MPC predicted trajectory 
                    // already filled above
                    //vector<double> mpc_x_vals;
                    //vector<double> mpc_y_vals;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line

                    msgJson["mpc_x"] = greenLineX;
                    msgJson["mpc_y"] = greenLineY;

                    //Display the waypoints/reference line
                    // already filled above
                    //vector<double> next_x_vals;
                    //vector<double> next_y_vals;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line

                    msgJson["next_x"] = yellowLineX;
                    msgJson["next_y"] = yellowLineY;

                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    
                    //std::cout << "sending message: " << msg << "\n\n";

                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to drive
                    // around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
                    // SUBMITTING.
                    // TODO: enable line below after debugging!
                    this_thread::sleep_for(chrono::milliseconds(100));

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }
            else 
            {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    uWsHub.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
        size_t, size_t) 
    {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) 
        {
            res->end(s.data(), s.length());
        }
        else 
        {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    uWsHub.onConnection([&uWsHub](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) 
    {
        std::cout << "Connected!!!" << std::endl;
    });

    uWsHub.onDisconnection([&uWsHub](uWS::WebSocket<uWS::SERVER> ws, int code,
        char *message, size_t length) 
    {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (uWsHub.listen(port)) 
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else 
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    
    uWsHub.run();
}
