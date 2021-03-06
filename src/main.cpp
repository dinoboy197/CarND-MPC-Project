#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>

#include <uWS/uWS.h>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "solution.h"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() {
  return M_PI;
}
double deg2rad(double x) {
  return x * pi() / 180;
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

void map_to_vehicle_coordinates(const double psi, const double px, const double py,
                                const std::vector<double> ptsx, const std::vector<double> ptsy,
                                std::vector<double> &vehicle_ptsx, std::vector<double> &vehicle_ptsy) {
  for (size_t i = 0; i < ptsx.size(); ++i) {
    double dx = ptsx[i] - px;
    double dy = ptsy[i] - py;
    vehicle_ptsx.push_back(dx * cos(-psi) - dy * sin(-psi));
    vehicle_ptsy.push_back(dx * sin(-psi) + dy * cos(-psi));
  }
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  const int latency_delay_in_ms = 100;
  const double latency_delay = latency_delay_in_ms / 1000.0;

  h.onMessage([&mpc, &latency_delay_in_ms, &latency_delay](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
      uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    auto sdata = std::string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      auto s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        auto event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          const std::vector<double> ptsx = j[1]["ptsx"];
          const std::vector<double> ptsy = j[1]["ptsy"];
          const double px_current = j[1]["x"];
          const double py_current = j[1]["y"];
          const double psi_current = j[1]["psi"];
          const double steering_angle_current = j[1]["steering_angle"];
          const double throttle_current = j[1]["throttle"];
          const double v_mph_current = j[1]["speed"]; // velocity in mph
          const auto v_mps_current = 1609.34 * v_mph_current / 3600.0; // velocity in meters per second

          // convert waypoints in map coordinates to those in vehicle coordinates
          std::vector<double> vehicle_ptsx;
          std::vector<double> vehicle_ptsy;
          map_to_vehicle_coordinates(psi_current, px_current, py_current, ptsx, ptsy, vehicle_ptsx, vehicle_ptsy);

          // Fit to curve with 3rd order polynomial
          const auto coeffs = polyfit(Eigen::VectorXd::Map(vehicle_ptsx.data(), vehicle_ptsx.size()),
              Eigen::VectorXd::Map(vehicle_ptsy.data(), vehicle_ptsy.size()),
              3);

          // The current cross track error is calculated by evaluating at polynomial at x, f(x)
          // and subtracting y.
          // py and px are be zero after the map coordinate correction, otherwise
          // this would be polyeval(coeffs, px) - py
          const auto cte_current = polyeval(coeffs, 0);

          // Due to the sign starting at 0, the orientation error is -f'(x).
          // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
          // psi is zero after the map coordinate correction, otherwise this would be psi - atan(coeffs[1])
          const auto epsi_current = - atan(coeffs[1]);

          // now predict cte and epsi into the future
          const auto v_mps_future = v_mps_current + throttle_current * latency_delay;
          const auto cte_future = cte_current + v_mps_future * sin(epsi_current) * latency_delay;
          const auto epsi_future = epsi_current + v_mps_future * steering_angle_current * latency_delay / Lf;

          Eigen::VectorXd state(6);
          // the first three values (px_future, py_future, psi_future) would be non-zero if we had not translated
          // from map to vehicle coordinates
          // use future vehicle state to account for actuator latency
          state << 0, 0, 0, v_mps_future, cte_future, epsi_future;

          // Calculate steering angle and throttle using MPC.
          // Both are in between [-1, 1].
          const auto solution = mpc.solve(state, coeffs);
          const auto steer_value = solution->steering_actuation;
          const auto throttle_value = solution->throttle_actuation;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = - steer_value / deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory

          msgJson["mpc_x"] = solution->path_x;
          msgJson["mpc_y"] = solution->path_y;

          //Display the waypoints/reference line
          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          for (size_t i = 0; i < 100; i += 5) {
            next_x_vals.push_back((double)i);
            next_y_vals.push_back(polyeval(coeffs, (double)i));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(latency_delay_in_ms));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
      size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
      char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
