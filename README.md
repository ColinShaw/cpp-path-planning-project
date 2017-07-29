# Path Planning Project

This is the path planning project for term 3 of Udacity's self-driving car program.  This 
original repo is available [here](https://github.com/udacity/CarND-Path-Planning-Project) and the 
simulator is available [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

![Screenshot](/images/screenshot.png)



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
targetting the speed limit, which is roughly 22 m/s, we would want the interpolated values to be 
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
send the simulator the second through final points in the second frame.  The way I achieved this
was to simply detect when the points we have visited by looking at the size of the previous path
given to us by the simulator.  If there is a lot left, I just let it keep working through what
it has, whereas if it is below some size limit I append a new planned path segment to it.  The
logic for this can be found starting at line `xyz` of `main.cpp`.  The effect of this is to 
keep the previous path buffers full and with appropriate points that make smooth paths.



## Trajectory and Control Types

For the sake of simplicity, a few types were created to help convey data about the telemetry
data for the car we are controlling as well as the sensor fusion data from the other cars.  What
is needed is simply a containing structure for the car and fusion data.  This is simply called
`telemetry_t` as seen on line `xyz` of `main.cpp`.  This contains a vector of `other_car_t` 
data (line `xyz` of `main.cpp`).  One thing to note is that these structures do not contain 
all of the data presented in the telemetry from the simulator.  The `telemetry_t` stucture is 
constructed on line `xyz` of `main.cpp`, and is used extensively when dispatching to computational
units after determining the best action (for example line `xyz` of the `costOfLaneChangeRight` 
function in `main.cpp`).

After a decision is made regarding the next action, the controlling variables must be determined
and conveyed to the minimum jerk trajectory function.  The natural way to encapsulate this is
also with a struct, in this case `setpoint_t` seen on line `xyz` of `main.cpp`.  Here we are 
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

Some example code can be found in the `/python` directory of the project.  The reason this
seemed like a good idea was first because it made sense given the zero acceleration 
constraint (until it proved not to minimize jerk), and also because it simplifies the 
system of equations to such a degree that there would not need to be reliance on 
a linear algebra library for computing coefficients.  This would simplify the code and
reduce a dependency.

However, it turns out that while the simpler model supports acceleration, it is not 
sufficient to meet the requirements of a starting and ending velocity with zero acceleration 
at the start and end.  That said, I ended up using the normal jerk minimizing 5th order 
polynomial.  The jerk minimizing trajectory generation is performed in the 
`minimum_jerk_path` function on line `xyz` of `main.cpp`.



## Making the Path Cycle

One goal of course is to be able to associate points beyond the end of the 
track with the equivalent point at the beginning of track.  This is actually
rather easy to accomplish in Frenet coordinates by simply taking `fmod` of the
total track length in s coordinates.  To simplify the code and not require
specifically wrapping the data, I appending the first line of the `highway_map.csv` 
file to the end, with the s coordinate of zero replaced with the track length 
(6945.554), so that it would wrap as expected.



## Computing Control Moves 

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
car in front of us and adjust our speed.  The way I chose to accomplish this was
to simply modify the planned path final speed proportional to a measure of 
distance from the car nearest in front of me.  This is pretty simplistic (though
seems to mimic the behavior of a lot of human drivers), but works quite 
effectively without requiring prior state of relevant cars to be retained.  This
can be seen in the code in the `XYZ` function on line `xyz` of `main.cpp`.

Once the respective costs have been computed and the lowest cost determined, 
the path constraints are created, including changes in speed to reflect the speed of the 
car in front of us, the constraints are sent to the minimal jerk trajectory code.  An
example of this for the straight path case can been seen in the 
`determineNewLeftCourseSetpoints` function on line `xyz` of `main.cpp`.



## The Car in Action

Here is a video of the car driving in action (image is a link):

[![Project video](https://img.youtube.com/vi/TGtL9_YQTbo/0.jpg)](https://www.youtube.com/watch?v=TGtL9_YQTbo)



## Performance Review

As can be seen in the video, the car drives around the track without issue. The
driving is smooth, it doesn't violate the requirements, and it makes it around
the track generally driving slightly under the speed limit unless it has to
slow down for a car in front.  Given the time horizon of the moves, which is
selected for smooth transitions that meet the acceleration and jerk objectives,
the path that is planned for lane changes is not all that aggressive.  

I did notice there are a few surprises that can arise where the the car does not 
go around the track without incident.  In a couple runs, a car from another lane 
on our side of the road swerved into my lane and caused a collision.  I don't 
think there is a passing way of dealing with this.  I cannot detect that and 
react without violating the speed limit, acceleration or jerk, or having 
the collision.  As well, a car from the other direction has swerved into the 
lane and collided with me.  In this case we don't even have the sensor fusion
data and clearly cannot do much about it.  

There seems to be a small section of track about 2/3 of the way through where 
there is a slight disconnect between the simulator's perceived location of the
car and the visible map.  On occassion the car can visibly be well within the
(rightmost) lane and be flagged as not being in the lane.  Since all of the
detection is done on the simulator side, I do not believe there is anything
I can do to accommodate this.  It seems that the simulator sometimes has 
residual path stored if you press `esc` to reset the track.  This can lead
to a shaky start, but if the simulator is restarted it does not exhibit
this behavior.



## Challenges Encountered

Below is an enumeration of the biggest challenges faced with implementing
the project:

 * Smooth waypoints
 * Smooth simulator interface
 * Tuning the minimum jerk path planner
 * Tuning the costs for the action planner
 * Cycling back at end of track

Most of these issues have been discussed above.  The waypoint smoothing was
essential because of the sudden acceleration and jerk encountered mapping
a minimum jerk trajectory through the supplied waypoints.  Obviously the 
interface with the simulator needs to be making predictions having continuity
with the existing predictions, which requires some finesse with asynchronous
entry.  The main goal of the minimizing acceleration and jerk while being 
able to change lanes required some tuning; decisions had to be made with 
regard to what the time horizon is that allows for smooth enough acceleration 
and jerk to meet the project requirement while also being able to competently 
weave in the traffic.  This relates to the decisions required for tuning 
the costs for the action planner, which also involved picking a reasonable 
collection of states that would admit a decent solution.  Cycling at the 
end of the track is no problem if `fmod` is used and the map waypoint data 
extended to associate the last point with the first.



## Code Style

There isn't much to say about the code style for this project.  It has utterly
minimal abstraction and is very direct.  One reason for this is the general
simplicity of the control flow.  There really is not a lot of value in abstracting
aspects of this project other than organization.  What I did do was clean up the
initial code given to us, remove aspects that were unused or unnecessay, and 
adhere to a clean style for what I added.  Yes, I know I do not have idiomatic
C++ styling, but I really like aligned `=` for ease of seeing what things are.

Between starting the project and finishing the project, I have tried out a 
variety of different organizations, but it is a size of project where some of 
the abstractions, like pulling minimum jerk functionality into a separate class,
mostly serves oganizationally to reduce file size, and doesn't really help
the readability and maintainability of the project.  It simply makes it easier
to find the aspects of the code that you don't know where it is or how it 
relates.  That said, I consciously chose to develop it as a large single file
project. 

