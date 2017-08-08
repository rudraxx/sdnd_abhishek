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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

## Reflections:
* For my project, I have implemented both the PID controller for
the crosstrack error (cte), and the velocity control.

* I have set the velocity to 40 mph. I have tested this at 30, and 50 mph. At higher speeds, the controller goes unstable. I need to look closer at the system dynamics and the closed loop transfer function to understand this better. It also makes sense, since the Psy_dot is proportional to the Vx * tan(delta_f) . So, for higher Vx, we need to either a) have a different set of pid values, or b) tune the controller for nominal conditions. I will be doing this in MATLAB/Simulink.

### Describe the effect of each PID component on the implementation

Here is what my output looks like:
![alt text](./images/pid_40mph.gif)
```
PID values for steering controller: (0.07,0.001,0.8);

PID values for Throttle Controller: (0.06,0.001,0.9);
```
* P term:
P term is acting directly on the cross track error. Increasing this reduces the response time, and so the controller responds faster. The issue with a large value is that it can overshoot by a large margin, and thus lead to instability.

* I term:
I term is acting on the total integral/summation of the error. i.e it is a measure of how far from the desired value have we been since the dawn of time. Using the I term leads to 0 steady state error. This really depends on the type of the system, and is based on the final value theorem from control theory. But atleast for a step input, we can expect a 0 steady state error, since the I term introduces a 1/s term at the origin.

Care must be taken to ensure that we don't increase this to a large number, since I haven't set up the anti-windup mechanism for the controls.

* D term:
D term acts on the rate of change of the error. The effect of D term is to add damping to the system, and reduces the amount of overshoot. Issues with a high D term come in when we have sensor noise in the system.
In the case of the simulator, we don't have a lot of noise, so the noise error isn't amplified, but a large value of D can cause spikes in the controller output, which is bad.

### Describe how the final hyperparameters were chosen.
* I chose the final parameters using a manual tuning. Starting with all values at 0, I increased the P term till I saw a bit of oscillatory behavior. Then I added a little bit of D term to dampen these oscillations.

* I increased the P and D term till I did't cause the system to go unstable.

* At this point, I added the I term to reduce the steady state error.
