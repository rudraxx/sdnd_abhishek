#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 10;
double dt = 0.1;
// specify the starting index for various vars.
int x_start     = 0*N;
int y_start     = 1*N;
int psi_start   = 2*N;
int v_start     = 3*N;
int cte_start   = 4*N;
int epsi_start  = 5*N;
int delta_start = 6*N;
int a_start     = 6*N + 1*(N-1);

double ref_v = 50*1.6*5/18; // ref velocity in m/s
// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// // This is the length from front to CoG that has a similar radius.
// const double Lf = 2.67;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // Step 1: Specify the cost function for this optimization:
    // cost is stored in first element of fg vector
    fg[0] = 0;
    // Cost function
    // Add cost associated with states cte, and epsi
    for (int t = 0; t<N; t++){
      fg[0] += (1.0/2.25)*900*CppAD::pow(vars[cte_start+t],2);
      fg[0] += (1.0/0.5625)*300*CppAD::pow(vars[epsi_start+t],2);
      fg[0] += (1.0/625.0)*40*CppAD::pow(vars[v_start+t]-ref_v,2);
    }

    // Add cost associated with use of actuators
    for (int t=0; t<N-1; t++){
      fg[0] += (1.0/0.16)*3*CppAD::pow(vars[delta_start+t],2);
      fg[0] += (1.0/1.0)*2*CppAD::pow(vars[a_start+t],2);
    }
    // Add cost associated with change in actuator inputs
    for (int t = 0; t<N-2; t++) {
      fg[0] += (1.0/0.64)*3*CppAD::pow(vars[delta_start+t+1] - vars[delta_start+t],2);
      fg[0] += (1.0/4)*4*CppAD::pow(vars[a_start+t+1] - vars[a_start+t],2);
    }

    // for (int t = 0; t<N; t++){
    //   fg[0] += 50*CppAD::pow(vars[cte_start+t],2);
    //   fg[0] += 50*CppAD::pow(vars[epsi_start+t],2);
    //   fg[0] += 0.25*CppAD::pow(vars[v_start+t]-ref_v,2);
    // }
    //
    // // Add cost associated with use of actuators
    // for (int t=0; t<N-1; t++){
    //   fg[0] += 10*CppAD::pow(vars[delta_start+t],2);
    //   fg[0] += 5*CppAD::pow(vars[a_start+t],2);
    // }
    // // Add cost associated with change in actuator inputs
    // for (int t = 0; t<N-2; t++) {
    //   fg[0] += 10*CppAD::pow(vars[delta_start+t+1] - vars[delta_start+t],2);
    //   fg[0] += 10*CppAD::pow(vars[a_start+t+1] - vars[a_start+t],2);
    // }

    // Step 2: Initialize the constraints for this iteration:
    // NOTE: We will be setting up the constraint boundaries in the MPC::Solve function.
    // Think of this as setting up the constraint equations themselves.

    // Specify the init state constraints in the fg vector.
    fg[1+x_start] = vars[x_start];
    fg[1+y_start] = vars[y_start];
    fg[1+psi_start] = vars[psi_start];
    fg[1+v_start] = vars[v_start];
    fg[1+cte_start] = vars[cte_start];
    fg[1+epsi_start] = vars[epsi_start];

    // Specify the remaining constraint equations/ fg values
    for (int t=1;t<N; t++){
      // psi, v, delta at time t-1, i.e prev timestep
      AD<double> x0     = vars[x_start+t-1];
      AD<double> y0     = vars[y_start+t-1];
      AD<double> psi0   = vars[psi_start+t-1];
      AD<double> v0     = vars[v_start+t-1];
      AD<double> cte0   = vars[cte_start+t-1];
      AD<double> epsi0  = vars[epsi_start+t-1];

      AD<double> delta0 = vars[delta_start+t-1];
      AD<double> a0     = vars[a_start+t-1];

      // States at t
      AD<double> xt    = vars[x_start+t];
      AD<double> yt    = vars[y_start+t];
      AD<double> psit  = vars[psi_start+t];
      AD<double> vt    = vars[v_start+t];
      AD<double> ctet  = vars[cte_start+t];
      AD<double> epsit = vars[epsi_start+t];

      // Pre calculate some values
      AD<double> fx0     = coeffs[0]+coeffs[1]*x0 +coeffs[2]*x0*x0+coeffs[3]*x0*x0*x0;
      AD<double> psi_des = CppAD::atan(coeffs[1]+2*coeffs[2]*x0+3*coeffs[3]*x0*x0 );

      // How states changes
      fg[1+x_start+t]     = xt    - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1+y_start+t]     = yt    - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1+psi_start+t]   = psit  - (psi0 + (v0/Lf) * delta0 * dt);
      fg[1+v_start+t]     = vt    - (v0 + a0 * dt);
      fg[1+cte_start+t]   = ctet  - (fx0 - y0 + (v0*CppAD::sin(epsi0)* dt));
      fg[1+epsi_start+t]  = epsit - (psi0 - psi_des + (v0/Lf) * delta0 * dt);

    }

  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Step 1: Initialize the size of variables and constraints based on the prediction horizon.

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = 6*N + 2*(N-1);
  // std::cout<< "n_vars=" << n_vars<< std::endl;

  // TODO: Set the number of constraints
  size_t n_constraints = 6*N;
  // NOTE: 6 constraints for the init conditions, and 6*(N-1) constraints for the dynamic eqns constraints.
  // Dynamic eqns constraints are (N-1) because if we have 2 time steps prediction horizon, the only constraints that come into effect are
  // x(k+1) = A* x(k) + B*u(k), x(k+2) is beyond the prediction horizon.
  // Which means that N is NOT the number of predicted states into the future, rather the current state, + N-1 predicted states.

  // std::cout<< "n_constraints=" << n_constraints<< std::endl;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  double x    = state[0];
  double y    = state[1];
  double psi  = state[2];
  double v    = state[3];
  double cte  = state[4];
  double epsi = state[5];

  Dvector vars(n_vars);
  for (i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }
  // Set values of first timestep as init values, and let all other stay 0.
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;
  // std::cout<< "delta_start = "<< delta_start << std::endl;

  // Step 2: Set the upper and lower bound for the various variables.
  // For the states, let us assume they can take any value
  // For the actuator commands, we can specify the min/ max of what the actuator is capable of doing.
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  // Set lower bound on the value of delta, +- 25 degrees. Set in radians.
  for (i = x_start; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  for (i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }
  for (i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Step 3: specify the bounds for the constraints.
  // NOTE: For the first constraint, which is associated with init state, (see Note in Step 1):
  // we want that to be the init value we specified in step 1. So min/max = init value for that state.
  // The remaining constraints represent the constraints for dynamics. So, ( x(k+1) - (A*x(k) + B*u(k) ) = 0

  // Lower and upper limits for the constraints
  // First, set all the constraints to 0, and then just modify the init state constraints.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0.0;
    constraints_upperbound[i] = 0.0;
  }
  // Set the constraints for the init states to be the values themselves.
  constraints_lowerbound[1+x_start] = vars[x_start];
  constraints_upperbound[1+x_start] = vars[x_start];

  constraints_lowerbound[1+y_start] = vars[y_start];
  constraints_upperbound[1+y_start] = vars[y_start];

  constraints_lowerbound[1+psi_start] = vars[psi_start];
  constraints_upperbound[1+psi_start] = vars[psi_start];

  constraints_lowerbound[1+v_start] = vars[v_start];
  constraints_upperbound[1+v_start] = vars[v_start];

  constraints_lowerbound[1+cte_start] = vars[cte_start];
  constraints_upperbound[1+cte_start] = vars[cte_start];

  constraints_lowerbound[1+epsi_start] = vars[epsi_start];
  constraints_upperbound[1+epsi_start] = vars[epsi_start];

  // Step 4: Create the fg_eval object. This is used for calling the nonlinear optimization.
  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  // for (int i=a_start;i<n_vars;i++){
  //   std::cout<< "Accel values are : "<< solution.x[i];
  // }
  // std::cout<<"end"<<std::endl;

  vector <double> result;
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);

  for (int i=0;i<N-1;i++){
    result.push_back(solution.x[x_start+i+1]);
    result.push_back(solution.x[y_start+i+1]);
  }
  return result;

}
