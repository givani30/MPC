# MPC Assignment
The goal is to simulate the behavior of a car and then implement obstacle detection and avoidance using model predictive control.
#WAT HEBBEN WE NODIG?
- Hoofd live script voor vicky
- COST FUNCTIE VOOR VERSLAG
- [A,B,C,D] ss(x,u) (discreet voor MPC en continue voor simulatie)
- Adaptive MPC
    - Constraints
        -Max steering angle
        -Max throttle
- Obstacle detection -> Constraint generator
- Plotting functie

## Requirements
To run this simulation, you will need:
- MATLAB (version R2017b or later)

## How to Run the Simulation
1. Clone or download the repository to your local machine.
2. Open MATLAB and navigate to the directory where you cloned/downloaded the repository.
3. Open the file `mpc_simulation.m`.
4. Run the script.

The simulation will start and display the behavior of the car. The obstacle detection and avoidance using model predictive control will be implemented after the simulation of the car's behavior has been completed.

## Obstacle Detection and Avoidance using Model Predictive Control
After the simulation of the car's behavior has been completed, the script will implement obstacle detection and avoidance using model predictive control. The obstacles will be simulated as rectangles on the road, and the MPC controller will calculate the optimal trajectory for the car to avoid them.

## Output
The simulation will output a graph of the car's trajectory and the obstacles on the road, as well as a video of the simulation.

## Parameters
The following parameters can be adjusted in the `mpc_simulation.m` file:
- `car_length`: The length of the car in meters.
- `car_width`: The width of the car in meters.
- `num_obstacles`: The number of obstacles on the road.
- `obstacle_size`: The size of each obstacle in meters.
- `horizon`: The length of the prediction horizon for the MPC controller.
- `sampling_time`: The time between control updates for the MPC controller.
- `max_speed`: The maximum speed of the car in meters per second.
- `max_acceleration`: The maximum acceleration of the car in meters per second squared.
- `max_steering_angle`: The maximum steering angle of the car in degrees.

## Credits
This simulation was created by Givani Boekestijn & Vicky van Heijningen
