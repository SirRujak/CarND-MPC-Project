# PID Control of a Simulated Vehicle

## Compilation

The easiest way to compile this code is to use the Udacity provided docker environment:

```
 sudo docker run -it -p 4567:4567 -v 'pwd':/work -v PATH_TO_PROJECT:/work/code udacity/controls_kit:latest

```


The code in this repository then comiples through the following process:

1. Clone this repo.
2. Start the docker container from the root of the project.
3. Make a build directory: mkdir build && cd build
4. Compile: cmake .. && make
5. Run it: ./mpc.

Instructions to install docker can be found here:

https://docs.docker.com/install/#supported-platforms


## Implementation

The code implements a Model Predictive Controller(MPC) with manual hyperparameter tuning. It compiles with cmake and make.

### The Model

The MPC model uses a combination of current state information, finite interval prediction, and actuator control to model
complex systems. Following is a description of each portion of the model:

#### State

The state of this MPC consists of the position of the car(x and y), the angle from a reference path(psi), the current
velocity of the vehicle(v), the current error from the predefined path(cte), and the current error from the paths angle(epsi).

This state is passed into the MPC for each prediction which is then used to predict out a locally optimal path.

#### Prediction

After pre-processing the state vector, the state is then passed into the MPC::Solve function along with a set of future
path points. Upper and lower bounds are set for each of the actuators at the maximum allowed values for the vehicle while
future state values were bounded to zero and non-actuator bounds to the equivilant of infinity.

The FG eval is then called to calculate a cost function based on the following:

    1. cte, epsi, and v errors
    2. minimal changes in steering and acceleration
    3. minimal change between sequential actuations
    
Each of these cost elements were then scaled through trial and error to obtain a functional MPC. The FG function then
calculates final model constraints for each prediction step. The set of calcluated bounds are then passed to the Ipopt
library to solve for a polynomial that minimizes cost while remaining within the given bounds.

#### Actuators

The designed MPC system outputs two actuators: a steering value and an acceleration value. Each of these actuators are
returned by the Ipopt solver which can then be directly passed back to the simulator.

### Timestep Length and Elapsed Duration (N & dt)

The number of timesteps considered(N) and the time between timesteps(dt) was chosen through a process of trial and error.
An initial value of N = 10 and dt = 0.1 were chosen due to information in the course. This ended up being the second best
solution found. Other values tried included

1. N = 25, dt = 0.04 in which the vehicle began prioritizing vehicle speed far too much and led to large cte values.
2. N = 10, dt = 0.2  in which the vehicle settled on a very low speed value but gave a very smooth turning profile.
3. N = 5,  dt = 0.2  in which the algorithm gave very bad results, this is expected to be due to having too few prediction
points
4. N = 10, dt = 0.15  which was found to have a good average of smooth and fast driving. 

### Polynomial Fitting and MPC Preprocessing

The vehicle state and waypoints are preprocessed to be in the reference frame of the car. This allows the state to always
have a zero x, y, and psi value and simplifies later calculations.

### Model Predictive Control with Latency

Latency is dealt with by predicting out the state vector of the car to the expected latency in the system. This is
specifically calculated by updating the x, y, psi, and v values using the latency, steering value, and angle before
the latency occurs.

## Simulation

### The vehicle must successfully drive a lap around the track.

The vehicle can successfully drive around the track for at least ten laps as per testing.
