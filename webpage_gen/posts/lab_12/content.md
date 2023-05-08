## WIP

This lab is still in progress, and the writeup is not yet complete. Come back soon for more.

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

## Results

## Conclusion

