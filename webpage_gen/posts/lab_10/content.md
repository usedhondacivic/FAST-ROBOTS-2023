## WIP

This lab hasn't been completed yet, come back later for more :)

## Introduction

In this lab I used a bayes filter to localize the robot in a simulated environment.

## Prelab

Before attempting the lab, I needed to get the simulator running on my computer. I followed the [installation instructions](https://cei-lab.github.io/FastRobots-2023/FastRobots-Sim.html) for the Ubuntu partition on my laptop. The only issue I encountered was Ubuntu complaining that the Box2D pip wheel complained it didn't support my OS, so I just installed from source. With the simulator running, I learned about the functions by implementing closed loop wall avoidance. Because my real robot has two sensors to the front of the robot, I edited /config/world.yml to include two sensors:

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