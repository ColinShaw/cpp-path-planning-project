# Path Planning Project

This is the path planning project for term 3 of Udacity's self-driving car program.  This 
original repo is available [here](https://github.com/udacity/CarND-Path-Planning-Project) and the 
simulator is available [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

    IMAGE GOES HERE



## General Project Overview

Solving the problem of driving the car smoothly around the track comes down to three 
sub-parts:

 * Smoothing the existing waypoint data 
 * Generating minimum jerk trajectories based on a minimal set of controls
 * Determining the best action based on model cost analysis

Throught the rest of this document I will describe some of the ways these are implemented
with code citations.



## Smoothing Waypoints

The track is approximately 7 km and only has 181 waypoints.  The simulator appear to use linear
interpolation between the waypoints to compute map coordinates from Frenet coordinates.  If you 
simply constrain the Frenet d-coordinate and sequentially increment the Frenet s-coordinate to 
drive the car, it will have too much acceleration and jerk at the waypoints.  There are a variety 
of ways to accommodate this; the way I decided to use involved interpolating the waypoints so 
that the entire Frenet frame could be assumed to be smooth.  Since the car is generally 
targetting the speed limit, which is roughly 25 m/s, we would want the interpolated values to be 
somewhat more fine than this.  I used an interpolation of 10,000 waypoints, which is a bit more 
fine than one waypoint per meter.  The curve is not always centered in the lane, an artifact of 
the interpolation, but it is more than adequate from the perspective of staying in the lane.  Now
the interpolation in converting Frenet coordinates to map coordinates is based on linear segments 
that are more smoothly varying with lengths less than one meter.

The code for this can be seen prior to the main execution loop, as the waypoint references are 
passed to the event handler.  This code starts at line `xyz` of `main.cpp`.



## Integrating with the Simulator Smoothly

The simulator consumes vectors of map points, moving the car through each point specified every 
20 ms.  Since the simulator operates asynchronously with the path planning code, there is some
finesse that is required to ensure the path data is given to the simulator properly.  The 
critical problem area is the handoff from one computed path segment to the next.  The issue
is getting the transitional points correct, as mismatch of the points to drive through makes
for transients in driving speed.  

The way that this is solved is being very specific about what the last coordinate we have sent 
and observing the coordinates processed by the simulator.  The goal is that we need to use the 
last coordinate from the first frame as first point of the next frame, though we only need to
send the simulator the second through final points in the second frame.  In order to effect 
this, we need to be able to retain state spanning entrance to the event loop.  To do this, I 
simply created the `save_state_t` type (line `xyz` of `structs.h`).  This is used in the code
of `main.cpp` at lines `xyz` and `xyz`, having to do with initialization and continuity of the
trajectory frames.



## Trajectory and Control Types

For the sake of simplicity, a few types were created to help convey data about the telemetry
data for the car we are controlling as well as the sensor fusion data from the other cars.  What
is needed is simply a containing structure for the car and fusion data.  This is simply called
`telemetry_t` as seen on line `xyz` of `structs.h`.  This contains a vector of `other_car_t` 
data (line `xyz` of `structs.h`).  One thing to note is that these structures do not contain 
all of the data presented in the telemetry from the simulator.  The `telemetry_t` stucture is 
constructed on line `xyz` of `main.cpp`, and is used extensively when dispatching to computational
units after determining the best action (for example line `xyz` of the `costOfLaneChangeRight` 
function in `main.cpp`).

After a decision is made regarding the next action, the controlling variables must be determined
and conveyed to the minimum jerk trajectory function.  The natural way to encapsulate this is
also with a struct, in this case `setpoint_t` seen on line `xyz` of `structs.h`.  Here we are 
again making assumptions about the data that we will be using and assigning these values 
accordingly.  In both this case and the telemetry structures, we are using the lane lines as 
integers rather than explicitly using the Frenet d-coordinate for simplicity.  This makes the 
grammar of selecting the next action simpler. 



## Computing Minimum Jerk Trajectories

The paths that are fed to the simulator are all segments computed to minimize jerk.  One
reason for this is because on any path, the minumum jerk interpolating polynomial 
produces data that is synchronized properly with time.  That is to say, it is the nature
of the polynomial we are optimizing for to have functional dependence on a time variable
that leads to smooth paths.  If we were to, say, use a spline to interpolate, it is not
one of the conditions of the optimization to enforce the same type of smoothness.

There are a couple of assumptions that are made to produce the smoothest paths.  First,
there is a constant time interval that is used for all actions.  This is long enough to
accomplish a smooth lane change, and determines some properties of the cost functions
used for selecting state in that the time horizon matters.  Second, there is the assumption
made that the acceleration at the beginning and the end of the minimum jerk frame is 
zero.  This simply assists us making paths where the transition from one frame to the
next does not experience jerk.  The velocity has the constraint that the velocity at the 
end of one frame must match the velocity at the beginning of the next frame, with 
implications having to do with both acceleration and jerk.

I tried some alternative models to the standard jerk minimizing 5th order polynomial 
model given the added constraint of zero acceleration at the frame boundaries.  Below
are the equations that govern this simplification as a 3rd order polynomial:

![3rd order polynomial](/images/3rdOrder.png)

... leading to four equations in four unknowns...

![3rd order polynomial system](/images/3rdOrderSystem.png)

Some example code can be found in the `/python` directory of the project.  It turns out 
that while the model supports acceleration, it is not sufficient to meet the requirements 
of a starting and ending velocity with zero acceleration at the start and end.  That said,
I ended up using the normal jerk minimizing 5th order polynomial.  The jerk minimizing 
trajectory generation is performed in the `minimum_jerk_path` function on line `xyz` 
of `main.cpp`.



## Computing Discreet Moves 

The telemetry data (described above) is used in computing what the next logical move
for the car to make is.  Given the relatively low variance in speeds of the other
cars and the relatively large gaps, it did not seem necessary to overcomplicate 
this with too many states.  I decided to use the following states:

 * Keep going straight
 * Lane change left
 * Lane change right

In the case of the lane changes, clearly the lane has to be appropriate (can't change 
left out of the leftmost lane) and it has to be acceptable with respect to the 
other cars in the lane.  To both as a cost, if the lane change is not appropriate
the cost is simply set high.  The positions of the nearest car (in the candidate lane)
in front of and behind our car are used to compute the cost for these cases.  An
example of this code can be seen in the `costOfLaneChangeLeft` function on line
`xyz` of `main.cpp`.

The cost for going straight is computed in a similar fashion, though considering 
only the car in the same lane that is directly in front of it.  This is because 
the other cars are fairly good about not hitting you from behind.  This code can
be seen in the `costOfStraightCourse` function on line `xyz` of `main.cpp`.

One additional challenge exists for the case of going straight, which is the need 
to maximize speed with the constraints of the speed limit and not running into the
car that is directly ahead of us.  The case could arise of not being able to change 
lanes, but the car in front of us is going slow, so it is necessary to monitor the 
speed of the car and estimate the speed.  To accomplish this we again use the 
`save_state_t` struct as a container for the prior car `id` and Frenet
s-coordinate.  The code for this can be found in the `estimateSpeedOfCarInFront`
function on line `xyz` of `main.cpp`.

Once the respective costs have been computed and the lowest cost determined, 
the path constraints are created to send to the minimal jerk trajectory code.  An
example of this for the straight path case can been seen in the 
`determineNewLeftCourseSetpoints` function on line `xyz` of `main.cpp`.



## Performance Review




## The Car in Action

Here is a video of the car driving in action:

    LINK TO VIDEO



## Challenges Encountered




## Code Style

There isn't much to say about the code style for this project.  It has utterly
minimal abstraction and is very direct.  One reason for this is the general
simplicity of the control flow.  There really is not a lot of value in abstracting
aspects of this project other than organization.  What I did do was clean up the
initial code given to us, remove aspects that were unused or unnecessay, and 
adhere to a clean style for what I added.  I did, of course, pull out the types
to `structs.h` so that they were a more clear reference.

