# Model description 
## Overview

The project is to navigate the ego car to drive around a highway with other traffic at a required speed.   
It is required for the car to complete 4.32 miles without incident. Incidents include exceeding acceleration/jerk/speed, collision, and driving outside of the lanes.  
The model will receive inputs as below:
1. The highway waypoints in the highway_map.csv file, which are point spaced roughly 30 apart between each other
2. The ego car telemetry information which can be used as localization
3. Previous path data which can be used to smooth the path
4. Sensor fusion data of other cars with the positions, speed

Corresponding steps include:
1. Interpolating waypoints of the track based on highway waypoints
2. Create Vehicle object with status and parameters
3. Make predictions from sensor fusion data of other traffic
4. Identify best trajectory
5. Define the new path

## Interpolating waypoints

The interpolating waypoint with s is simply done by:
```c++
interpolated_waypoints_s.push_back(coarse_waypoints_s[0]);
for (int i = 1; i < num_interpolation_points; i++) {
   interpolated_waypoints_s.push_back(coarse_waypoints_s[0] + i * dist_inc);
}
interpolated_waypoints_x = interpolate_points(coarse_waypoints_s, coarse_waypoints_x, dist_inc, num_interpolation_points);
interpolated_waypoints_y = interpolate_points(coarse_waypoints_s, coarse_waypoints_y, dist_inc, num_interpolation_points);
interpolated_waypoints_dx = interpolate_points(coarse_waypoints_s, coarse_waypoints_dx, dist_inc, num_interpolation_points);
interpolated_waypoints_dy = interpolate_points(coarse_waypoints_s, coarse_waypoints_dy, dist_inc, num_interpolation_points);
```
And the coordinates transfer is done by function interpolate_points in smoother.h

## Create Vehicle object

The Vehicle (defined in vehicle.cpp and vehicle.h) object is initiated:
```c++
Vehicle my_car = Vehicle();
```
And parameters are set by:
```c++
my_car.s    = pos_s;           // s position
my_car.s_d  = s_dot;           // s dot - velocity in s
my_car.s_dd = s_ddot;          // s dot-dot - acceleration in s
my_car.d    = pos_d;           // d position
my_car.d_d  = d_dot;           // d dot - velocity in d
my_car.d_dd = d_ddot;          // d dot-dot - acceleration in d
```

## Make predictions from sensor fusion data

Other cars data are imported into Vehicle objects as well, and also the sensor fusion data; the prediction is done by generate_predictions function in Vehicle class:
```c++
vector<Vehicle> other_cars;
map<int, vector<vector<double>>> predictions;
for (auto sf: sensor_fusion) {
   double other_car_vel = sqrt(pow((double)sf[3], 2) + pow((double)sf[4], 2));
   Vehicle other_car = Vehicle(sf[5], other_car_vel, 0, sf[6], 0, 0);
   other_cars.push_back(other_car);
   int v_id = sf[0];
   vector<vector<double>> preds = other_car.generate_predictions(traj_start_time, duration);
   predictions[v_id] = preds;
}
```

## Identify best trajectory

1. calculate the possible states of ego vehicle
```c++
my_car.update_available_states(car_to_left, car_to_right);
```
2. Get target state annd generate all possible trajectory (functions are defined in Vehicle class)
```c++
vector<vector<double>> target_s_and_d = my_car.get_target_for_state(state, predictions, duration, car_just_ahead);
vector<vector<double>> possible_traj = my_car.generate_traj_for_target(target_s_and_d, duration);
```
3. Calculate cost of trajectery and find the best trajectery
```c++
double current_cost = calculate_total_cost(possible_traj[0], possible_traj[1], predictions);                 
if (current_cost < best_cost) {
    best_cost = current_cost;
    best_frenet_traj = possible_traj;
    best_traj_state = state;
    best_target = target_s_and_d;
}
```

## Get the new path

Use 5 waypoints of the previous two waypoint of the car from simulator, current position of the car, and two waypoints from 30 m and 60 m ahead in the target lane to get a smooth spline path that the car will drive. And to avoid excessive acceleration or jerk, the velocity is controlled by small amount of increment and decrement.


# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

