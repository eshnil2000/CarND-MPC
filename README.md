# MPC Control
Udacity Self-Driving Car Nanodegree Term 2, Project 5

## Project Basics
This project implements a C++ program that can drive a  car around a virtual track using specific waypoints from the track. The simulated car's actuators have a 100ms latency (delay) that must be accounted for as well as part of the MPC calculation.

## Project setup
* A simulator is provided which transmits waypoints[x,y co-ordinates, and the current steering angle,speed and throttle values] to this program via websocket. The simulator also receives via websockets, actuator values [steering angle, throttle and speed]. The goal of this project is to compute the optmaal actuator values to send back to the simulator to effectively reduce the CTE [Cumulative Error] between the desired waypoints and the actual x,y position of the car [calculated after the 100ms delay] as a result of the computed steering angle, speed and throttle values sent to the simmulator.

### Project Steps
* Fit a line based on road waypoints and evaluating the current state based on that polynomial line.
* Implement the MPC calculation, including setting variables and constraints
* Calculating actuator values based on current state
* Account for latency (100ms replace the actual current state in the calculation)
* Calculate steering angle & throttle/brake based on the actuator values
* Set timestep length and duration
* Test/tune of above implementations on Udacity simulator.

### Issues I ran into, tricky situations and key parameters
* On my Mac Pro, due to lack of sufficient compute power, the IPOPT solver was unable to compute results within a sufficient amount of time. After some experimentation, I was able to overcome this problem via setting a high value for the following parameter:   options += "Numeric max_cpu_time          100\n";
* The other tricky part was getting the right number of "lookahead" waypoints to use to compute the polynomial curve. After some experimentation, I settled down with int num_points = 10;
* The timestep length and duration, which impact the time to prediction and can slow down the computation, but also accuracy, Currently tuned to predict 1 second worth of data, were the next parameters that needed tuning. After some experimentation, I settled on the values used in the classroom videos, using 10 points and a timestep of 0.1s. size_t N = 10; double dt = 0.1;
* I set the max speed to 100 const double ref_v = 100; 
* The next challenge was to set the Weights for how "important" each cost is when setting up the cost equation for the optimizer. I chose to give weights in orders of magnitude differences: 1,10, 100, 1000. Higher the weight, the higher the importance of that parameter in the cost function. The Cumulative Total Error and the error in the orientation angle of the vehicle psi got the highest order of weights[2000], followed by the rate of change delta_change_cost_weight, to ensure no sudden jerks, then the change (delta) and acceleration were next with a weight of 10, finally the velocity was the least important parameter with a weight of 1. 
    const int cte_cost_weight = 2000;
    const int epsi_cost_weight = 2000;
    const int v_cost_weight = 1;
    const int delta_cost_weight = 10;
    const int a_cost_weight = 10;
    const int delta_change_cost_weight = 100;
    const int a_change_cost_weight = 10;

### Results
See video of the results from my implementation [here](mpc.mov).

## Discussion/Reflection
### The Model
The model starts out by taking in certain information from the simulator[json formatted data defined in DATA.md]: 
* `ptsx` (Array<float>) - The global x positions of the waypoints.
* `ptsy` (Array<float>) - The global y positions of the waypoints. This corresponds to the z coordinate in Unity
since y is the up-down direction.
* `psi` (float) - The orientation of the vehicle in **radians** converted from the Unity format to the standard format expected in most mathemetical functions (more details below).
* `psi_unity` (float) - The orientation of the vehicle in **radians**. This is an orientation commonly used in [navigation](https://en.wikipedia.org/wiki/Polar_coordinate_system#Position_and_navigation).
* `x` (float) - The global x position of the vehicle.
* `y` (float) - The global y position of the vehicle.
* `steering_angle` (float) - The current steering angle in **radians**.
* `throttle` (float) - The current throttle value [-1, 1].
* `speed` (float) - The current velocity in **mph**.

#### Polynomial Fitting & Preprocessing
To simplify the calculations, transform the points from the simulator's global coordinates into the vehicle's coordinates/ orientation. 
```
          for (int i = 0; i < ptsx.size(); i++) {
            double x = ptsx[i] - px;
            double y = ptsy[i] - py;
            ptsx_car[i] = x * cos(-psi) - y * sin(-psi);
            ptsy_car[i] = x * sin(-psi) + y * cos(-psi);
          }
```

* Using the `polyfit()` function, a third-degree polynomial line is fit to these transformed waypoints, this is drawing the yellow path the vehicle should try to travel.           
```
auto coeffs = polyfit(ptsx_car, ptsy_car, 3);
```

* px, py and psi all equal to zero: Since points we normalized to vehicle position, this is a valid calculation.
```
double cte = polyeval(coeffs, 0);
```
 * The psi error (vehicle orientation), or epsi, calculated from the derivative of polynomial fit line, is the negative arc tangent of the second coefficient (the first-order x was in the original polynomial).
 ```
 double epsi = -atan(coeffs[1]);
 ```

#### Actuator for Latency
* To account for latency, predict the state using motion equations after the desired latency period, instead of at the observed/reported x,y values by the simulator. 
```
const double dt = 0.1;
          
      // Predict state after latency
      // x, y and psi are all zero after transformation above
      double pred_px = 0.0 + v * dt; // Since psi is zero, cos(0) = 1, can leave out
      const double pred_py = 0.0; // Since sin(0) = 0, y stays as 0 (y + v * 0 * dt)
      double pred_psi = 0.0 + v * -delta / Lf * dt;
      double pred_v = v + a * dt;
      double pred_cte = cte + v * sin(epsi) * dt;
      double pred_epsi = epsi + v * -delta / Lf * dt;
```

Prior to sending predicted values to the simulator, dely is implemented using:
https://en.cppreference.com/w/cpp/thread/sleep_for
```
this_thread::sleep_for(chrono::milliseconds(100));
```
#### MPC.cpp - 
* This implements the main 2 functions/classes: 
* FG_eval & Solve. 

ipopt::solve solves nonlinear programming problems of the form:
```
Minimize f(x)
given glower_bound<g(x)<gupper_bound and xlower_bound<x<x_upper_bound
```
* This is done using Ipopt optimizer and CppAD for the derivative and sparsity calculations. 
* The function signature is:
```
ipopt::solve(
     options, xi, xl, xu, gl, gu, fg_eval, solution
) 
```
* xi is input vector, xl,xu - lower,upper bounds. These variables are the upper and lower bound values of each of the actuators.
* gl,gu- these are the upper and lower bounds of the contraints.

* The "solution" field returns:
* status, x- final optimizer output and other constraint function related values.
In this case, x contains the entire vector consisting of computed actuator values and the predicted x,y values to be passed to the simulator to print on the screen.

FG_eval class: 
This class sets up all the constraints into a single vector, along with the cost function as the first element of the vector "fg".

The signature of FG_eval is: fg_eval(fg, x)
In the program:
```
void operator()(ADvector& fg, const ADvector& vars)
```
`fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators) and fg[1+i]=gi(x)

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
