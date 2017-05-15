# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

## Reflection

### PID Components
A PID controller calculates an error value as the difference between a desired value and a measured process variable and applies a correction based on proportional, integral, and derivative terms. In this project two PID controllers are implemented to control the steering angle and the throttle separately. In general, the control output is calculated as:

`control_output = -total_error = -Kp * err - Ki * int_err - Kd * diff_err`

where `err` is the present error, `int_err` is the integration of all past errors, and `diff_err` is the difference between consecutive present and past error. `Kp`, `Ki`, and `Kd` are the coefficients of the present, integrated, and differential errors, respectively.

A [sigmoid function](https://en.wikipedia.org/wiki/Sigmoid_function) is used to regulate the control output values to between `-1.0` and `1.0`.

The three coefficients in the PID controller contribute to different effects:
* **P**: accounts for the present error. It is the most direct control on the vehicle to correct the current error. However, large value of this coefficient could cause overshoot.
* **I**: accounts for all history of the errors. It can eliminate the system biases if present. In this project this value should be very small as the simulator does not appear to have any big system bias.
* **D**: acounts for the difference of the consective errors. It can help reducing future trends of the error based on its current rate of change. This is very helpful to reduce overshoot.

### PID Parameter Tuning

The parameters of the PID controller is manually tuned for now. A twiddle algorithm will be implemented in a later version.

## Build and Run Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 
