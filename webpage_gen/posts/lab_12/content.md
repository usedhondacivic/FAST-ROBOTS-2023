## Introduction

In this lab I combined my work in all past labs to make my car intelligently follow a set of waypoints.

The waypoints are as follows:

```
1. (-4, -3)    <--start
2. (-2, -1)
3. (1, -1)
4. (2, -3)
5. (5, -3)
6. (5, -2)
7. (5, 3)
8. (0, 3)
9. (0, 0)      <--end
```

![The waypoints plotted](./assets/waypoints.png)

## Strategy

Note that all waypoints are reachable using a straight line path from the previous waypoint. Because my localization results are so promising, I chose to ignore obstacle avoidance and trust that I can accurately follow the straight line paths. The resulting routine is:

1. Localize
2. If localized point = waypoint, set next point as target waypoint.
3. Calculate angle and distance to target waypoint
4. Rotate to target angle
5. Move forward for time proportional to distance
6. Go to 1

By running through this process enough times, the robot will eventually navigate the entire path.

## Implementation

For my previous labs, I used Python as a commander for most of the robots actions. This works fine when doing isolated tasks (ie localizing one time), but can lead to accumulated errors when the tasks are chained together. A large part of the error comes from BLE communication delay, which is non-deterministic.

Instead of trusting the unreliable timing from the laptop, I implemented a series of modes for the Artemis to address timing sensitive routines. These routines are seek angle, travel straight for time, and get localization readings. Pieced together, they are all thats needed for the algorithm outlined in strategy.

## Results

I was unfortunately unable to finish this lab. The robot was able to successfully navigate two waypoints, then the motor drivers would overheat and cause the movement to become erratic. I hypothesize this had something to do with the rapid direction changes from the PID loop for angular velocity control. My laptop also decided to kick the bucket while debugging this issue, so thats all she wrote folks!

Thanks for a great semester to everyone involved. Live laugh fast robots.