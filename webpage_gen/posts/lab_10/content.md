## WIP

This lab hasn't been completed yet, come back later for more :)

## Introduction

In this lab I used a bayes filter to localize the robot in a simulated environment.

## Prelab

Before attempting the lab, I needed to get the simulator running on my computer. I followed the [installation instructions](https://cei-lab.github.io/FastRobots-2023/FastRobots-Sim.html) for the Ubuntu partition on my laptop. The only issue I encountered was Ubuntu complaining that the Box2D pip wheel didn't support my OS, so I just installed from source. With the simulator running, I learned about the functions by implementing closed loop wall avoidance. Because my real robot has two sensors to the front of the robot, I edited /config/world.yml to include two sensors:

```yaml
# Specify the angles (in a counter-clockwise direction) for the ToF sensor(s) where 0 degrees is at the robot's heading
angles: 
    - -20
    - 20
```

Below is my code and a video showing the results.

```python
cmdr.reset_plotter()
cmdr.reset_sim()

while cmdr.sim_is_running() and cmdr.plotter_is_running():
    pose, gt_pose = cmdr.get_pose()
    cmdr.plot_odom(pose[0], pose[1])
    cmdr.plot_gt(gt_pose[0], gt_pose[1])
    
    dist_right = cmdr.get_sensor()[0]
    right_contrib = -0.5/(dist_right * dist_right)
    dist_left = cmdr.get_sensor()[1]
    left_contrib = -0.5/(dist_left * dist_left)
    
    cmdr.set_vel(1, right_contrib + left_contrib)
```

<iframe width="492" height="875" src="https://www.youtube.com/embed/HD-ApRYbShs" title="ECE 4160 - Closed Loop Sim Control" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

## Theory Behind the Bayes Filter

The Bayes Filter is a probabilistic approach that calculates the distribution of some quantity over a discretized state space. In this case the quantity is the robot's pose (x, y, theta) and the discretized space is the map divided into 1 m squares with possible poses divided every 20 degrees. This translates into a 3D grid of size (12, 9, 19), which is small enough to compute over fully.

The Bayes filter algorithm is as follows:

![The Bayes Filter](./assets/bayes_filter.png)

Where $bel(x_{t-1})$ is the belief distribution before running the filter,
$u_t$ is the commands we gave the robot in this time-step,
and $z_t$ is the sensor measurements for this time-step.

The filter iterates over every pose $x_t$ and computes the probably that it's the true pose given the input variables. The first step is the prediction step, which finds how likely the state is given the previous state and our command. If we tell the car to move forward, we would expect the car to execute that command and actually move according to our understanding of it's dynamics. This should cause the distribution to move accordingly, and is factored into the calculation.

Using only the update step is equivalent to relying entirely on odometry, which previous experiments have shown is an unreliable approach. To solve this problem, we integrate the update step. The update step uses sensor measurements and a sensor model to compare the predicted sensor measurements for $x_t$ to the observed values. The result is a probability representing how likely it is that the sensor measurements were taken from that pose.

## Implementing the Bayes Filter in Python

As a start for my implementation, I used [Linda Li's](https://lyl24.github.io/lyl24-ece4960/lab11) implementation. I updated the model for my specific sensor arrangement, but unless noted the code was written by her.

