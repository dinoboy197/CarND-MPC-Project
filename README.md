# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Video of vehicle control

Please see a video of the MPC I implemented in this project controlling the steering of the vehicle in the Term 2 simulator [here](https://youtu.be/ExfV4IqEtCU).


## Implementation details

### The Model

The vehicle is modeled using a kinematic bicycle model, consisting of six elements:

* vehicle x position (`x`)
* vehicle y position (`y`)
* vehicle heading (`psi`)
* vehicle velocity (`v`)
* cross track error (`cte`)
* vehicle heading error (`epsi`)

Two actuator values are outputted by the MPC:

* steering (`steer`)
* throttle (`accel`)

The cost function for predictions includes a linear combination of seven components (each component is squared before being used, to ensure differentiability in the cost optimizer):

* cross track error
* vehicle heading error
* difference between current and target speed
* steering value (zero being steering straight; multiplied by factor of 2000)
* throttle value (zero being no acceleration nor braking)
* steering change rate (zero being no change to steering; multiplied by factor of 5)
* throttle change rate (zero being no change to throttle; multiplied by factor of 5)

The update equations for each of the states are:

* `x[t+1] = x[t] + v[t] * cos(psi[t]) * dt`
* `y[t+1] = y[t] + v[t] * sin(psi[t]) * dt`
* `psi[t+1] = psi[t] + v[t] / Lf * steer[t] * dt`
* `v[t+1] = v[t] + accel[t] * dt`
* `cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt`
* `epsi[t+1] = psi[t] - psides[t] + v[t] * steer[t] / Lf * dt`

where `Lf` is a simulation-based constant describing the difference between the front of the vehicle and its center of gravity.

The MPC predicts `N` state vectors and `N-1` actuation vectors using a mathematical optimizer ([Ipopt](https://projects.coin-or.org/Ipopt)). The first actuation vector is used for the each successive actuation of the vehicle (the following actuator values are not used); the state vectors are useful for debugging and visual optimization in the simulator.

Using the cost components above, the vehicle is able to drive smoothly around the simulated track at up to ~55mph without creating hazardous driving conditions.

### Timestep length and Elapsed Duration

A timestep length of `N` = 20 and elasped duration between timesteps of `dt` = 0.05 was initially chosen. During the parameter tuning phase of the project, it was found that this `dt` led to steering overcorrection of the vehicle, so the `dt` was doubled to 0.1 and `N` halved to 10. After that, the total length of the predicted course was found to be too short, too many back and forth steering corrections. By increasing the `N` = 20, the vehicle was able to drive around the path with enough future predicted states while not predicting sharp corrections.

### Polynomial fitting and MPC pre-processing

Waypoints from the simulator are received in map coordinates; before they can be used, they are converted from map coordinates to vehicle coordinates (using the map coordinates of the vehicle itself). These waypoints (now in vehicle coordinates) are then used to fit a 3rd degree polynomial. This polynomial is used to compute the desired location of the vehicle on the path; this allows for computing the cross track error as the distance between the desired location of the vehicle and the current location and the vehicle heading error.

### Model Predictive Control with Latency

Due to a 100ms simulated actuator delay between actuator value computation and actuator implementation, any actuator values computed by the MPC are 100ms out of date by the time they arrived at the simulated vehicle. Because of this, a latency adjustment is computed which, at each actuator computation, predicts the vehicle state into the future by the latency amount. This is done simply by using the same vehicle update equations above a single time before passing the state into the MPC itself.

With these adjustments, the MPC that was implemented easily handles a 100ms latency with the simulated vehicle driving at ~55mph.

### Code

This project was designed and implemented on Ubuntu Linux 16.04 with gcc version 5.4.0, and has only been tested and verified to work on such a platform. This code may or may not be interoperable on different platforms (MacOS or Windows, for example). Any evalation of this work should be done on a similar platform.

--

## Dependencies

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

## Basic Build Instructions

1. Clone this repo.
1. Compile: `cmake . && make`
1. Run it: `./mpc`.
