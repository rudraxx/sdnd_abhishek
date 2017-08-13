# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

For details on dependencies and stuff to install, see the README_old.md

## Considering the following aspects from the project rubric

### Compilation
The compilation works with cmake and make commands. Here is a video of the MPC in action:

![MPC video](./images/out.gif)

### Implementation
#### The model
* We are using a simple kinematic model for the MPC Implementation. These equations are used to update where the car will be in the future
These are used at 2 places, once in the main.cpp for compensating for the delay, and the second time in the MPC.cpp for updating the constraints.
Here is a snippet of code from the Main.cpp (line 107- 112)
```
 fg[1+x_start+t]     = xt    - (x0 + v0 * CppAD::cos(psi0) * dt);
 fg[1+y_start+t]     = yt    - (y0 + v0 * CppAD::sin(psi0) * dt);
 fg[1+psi_start+t]   = psit  - (psi0 + (v0/Lf) * delta0 * dt);
 fg[1+v_start+t]     = vt    - (v0 + a0 * dt);
 fg[1+cte_start+t]   = ctet  - (fx0 - y0 + (v0*CppAD::sin(epsi0)* dt));
 fg[1+epsi_start+t]  = epsit - (psi0 - psi_des + (v0/Lf) * delta0 * dt);
```
* The kinematic equations used here are not completely consistent with the literature. But since these are used in the Udacity simulator, any changes I make will adversely affect the system behavior., so decided to go with these.
[ Discussion on the Udacity Forum](https://discussions.udacity.com/t/incorrect-kinematic-model/329061/3)

* Not sure if using a more complex vehicle moel will help or not, since I don't know what has been implented in the simulator. It would be good to know if this kinematic model is what has been implemented there, or if it is something more complex.

#### Timestep Length and Elapsed Duration (N & dt)
* For the timesteps, typically 1 second of preview is good enough. Any more and the results will be meaningless since the kinematic model is just an approximation. In my case, I have chose a dt of 50 millisec, and N = 20. Real world image sensors also give back data (eg MobilEye) at same rates, so this dt works well.

* The other options I tried were N = 100, and dt = 10 milliSec . This caused too small a prediction horizon, and system quickly went unstable.

#### Polynomial Fitting and MPC Preprocessing
The function polyfit is  used for fitting a polynomial to waypoints.
```Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {}
```
I am compensating for the system delay by predicting the future states based on the current states. This enables us to use MPC for predicting on a future state.
``` // Code
px = px + velocity * cos(psi) * system_delay;
py = py + velocity * sin(psi) * system_delay;
psi = psi + (velocity/Lf) * steer_value * system_delay;
velocity = velocity + throttle_value * system_delay;

```
#### Model Predictive Control with Latency
As described in the above section, I am taking the delay into account before any of the MPC functions by calculating the future state of the system. This way the command reaches the simulator, it will be the *correct time* command and not a delayed command.
The other way is to add another 1 or 2 timestep delay states in the MPC state vector. This restructuring of the state vector will cause a full restructuring of the MPC equations. So, didn't do that.

### Simulation
No part of the car goes outside the drivable road.
