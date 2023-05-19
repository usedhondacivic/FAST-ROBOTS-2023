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

## Artemis Side

For my previous labs, I used Python as a commander for most of the robots actions. This works fine when doing isolated tasks (ie localizing one time), but can lead to accumulated errors when the tasks are chained together. A large part of the error comes from BLE communication delay, which is non-deterministic.

Instead of trusting the unreliable timing from the laptop, I implemented a series of modes for the Artemis to address timing sensitive routines. These routines are seek angle, travel straight for time, and get localization readings. Pieced together, they are all thats needed for the algorithm outlined in strategy.

```cpp
if(current_mode == TAKE_READINGS){
    if(mode_changed){
        // Record the starting rotation
        reading_start_rot = sensor_readings.gyro.z; 
        // Init PID 
        pid_controllers.pid[ROTATION].reset(); 
        pid_controllers.pid[ROTATION_RATE].reset();
        pid_controllers.setpoints[ROTATION] = reading_start_rot;
        pid_controllers.setpoints[ROTATION_RATE] = current_target;
        // Save readings
        data_buffers.enabled[POSE] = true;
        data_buffers.enabled[TOF] = true;
    }

    // Do a 360 to take readings
    if(sensor_readings.gyro.z - reading_start_rot < 360.0){ 
        float out = pid_controllers.pid[ROTATION_RATE].output;
        if(out >= -0.1){
            out = -0.1;          
        }
        set_wheel_output(out, -out);
    }else{
        data_buffers.enabled[POSE] = false;
        data_buffers.enabled[TOF] = false;
        // Return to angle from start of reading
        current_mode =  SEEK_ANGLE;
        current_target = reading_start_rot;
    }
}
```

The take readings mode uses rotation speed PID control to do a full 360, then sets the mode to seek angle to return the car to it's initial rotation. This is important because the Bayes filter returns and estimate of the car's pose at the beginning of the reading, and the navigation algorithm is based on that estimate. Note that `current_target` is a general use parameter that can be set from python side.

```cpp    
if(current_mode == SEEK_ANGLE){
    pid_controllers.setpoints[ROTATION] = current_target;
    float out = pid_controllers.pid[ROTATION].output;
    set_wheel_output(out, -out);
}
```

See angle uses PID control to orient the car to face the target angle.

```cpp
if(current_mode == MOVE_FORWARD){
    if(mode_changed){
        pid_controllers.setpoints[ROTATION] = sensor_readings.gyro.z;
        mode_start = millis();
    }
    float out = pid_controllers.pid[ROTATION].output;
    if(millis() - mode_start < 100){
        set_wheel_output(0.5 + out, 0.5 - out);
    }
    else if(millis() - mode_start < current_target){
        set_wheel_output(0.2 + out, 0.2 - out);
    }else{
        set_wheel_output(0, 0, true);
    }
}
```

Move forward condenses the logic for driving straight for an amount of time. I could accomplish the same thing using my previous command system on the python side, but the Python side timing was not consistent enough for reproducible distances. I also made the algorithm give the car a kick at the beginning to overcome static friction, which was introducing randomness into the distance.

## Python Side

### Saving Views to Increase Resolution

The straight line strategy I adopted requires precise localization to be effective. To get better readings, I doubled the resolution of the Bayes filter in all dimensions.

```yaml
# Discrete cell sizes
# It is recommended that you do not change these values, unless you truly know what you are doing
cell_size_x: 0.1524
cell_size_y: 0.1524 
cell_size_a: 5

# Number of cells in each dimensions(x,y,a)
# Calculated based on the range divided by the cell size in each dimension
# Ex: max_cells_x = (max_x-min_x)/cell_size_x
# Must be integers
max_cells_x: 24  # Total number of grid cells in the x direction
max_cells_y: 18   # Total number of grid cells in the y direction
max_cells_a: 72  # Total number of grid cells in the a direction
```

Increasing the number of views dramatically increased the precaching time, taking it from 10 seconds to around two minutes. Luckily, this computation based on the map and the sensor arrangement, neither of which will change. I edited the base localization code to save the output to a file, then loaded the data from the file on future runs.

It was as simple as writing the following code on line 211 of `localization.py`

```python
np.save('obs_views.npy', self.obs_views);
np.save('obs_points_x.npy', self.obs_points_x);
np.save('obs_points_y.npy', self.obs_points_y);
```

Then running the localization initialization once. After the data had been stored to a file, replace line 90 with:

```python
self.obs_views = np.load('obs_views.npy')
self.obs_points_x= np.load('obs_points_x.npy')
self.obs_points_y = np.load('obs_points_y.npy')
```

And you get (almost) instantaneous pre-caching of views for future runs!

### Implementing the Algorithm

As a reminder, here is the algorithm:

1. Localize
2. If localized point = waypoint, set next point as target waypoint.
3. Calculate angle and distance to target waypoint
4. Rotate to target angle
5. Move forward for time proportional to distance
6. Go to 1

I'll walk through my code step by step.

#### Initialization

Before beginning the actual algorithm, I have to initialize some variables. Note that I'm omitting the setup code for BLE, the simulator, and the Bayes filter. If you want the details of their setup, labs 10-11 go into detail. 

```python
# Reset Plots
cmdr.reset_plotter()

# Init Uniform Belief
loc.init_grid_beliefs()

# Store all of the waypoints
points = np.array([[-4, -3], [-2, -1], [1, -1], [2, -3], [5, -3], [5, -2], [5, 3], [0, 3], [0, 0]])

points = points * 0.3048 # Convert from feet to meters

target_index = 1
```

I reset the plot and create a uniform belief grid. Technically this is the incorrect move - the robot doesn't have a uniform belief. We know for a fact that it will start at the beginning of the path and therefore that bucket should have a belief of 1. However, the algorithm already doesn't include a prediction step so we are essentially creating a uniform belief before every update anyway. I also store all of the waypoints in order and convert them into meters.

#### Localization

We now enter the main loop, represented by step 6 in the algorithm before. From my work in [lab 11](../lab_11), a simple one liner with retrieve all of the observation data for me. I can then update the Bayes filter as normal.

```python
while True:
    # Get Observation Data by executing a 360 degree rotation motion
    await loc.get_observation_data()

    # Run Update Step
    loc.update_step()
    loc.plot_update_step_data(plot_data=True)

```

#### Checking Waypoints

Once the robot has localized, we can check if it is on top of it's target waypoint. If so, it increments the target and begins seeking that waypoint. This will cause an invalid index error when we land on the last waypoint, but at that point were already done and don't care.

```python
# Check if the robot is on its target
if (np.abs(target - pose_est[0:2]) < 0.3).all():
    print("Hit waypoint " + str(target))
    target_index+=1
    target = points[target_index, :]
    print("New target: " + str(target))
```

#### Find Angle / Dist to Waypoint

Knowing the current location and the target location, its easy to find the angle and distance. The angle is calculated using atan2 and then subtracting the estimated rotation. This gives a angle relative to the robots current direction.The distance is calculated is the L2 norm of target - pose.

```python
# Find the distance and angle to target
delta = target - pose_est[0:2]
dist = np.linalg.norm(delta)
print("Angle estimate: " + str(pose_est[2]))
angle = np.rad2deg(np.arctan2(delta[1], delta[0])) - pose_est[2]

print("Angle to target: " + str(angle))
print("Distance to target: " + str(dist))
```

#### Rotate Towards Target Angle
Because my gyroscope has significant drift (mostly because I haven't tuned it what-so-ever) I adjust the robot clockwise by 20 degrees to offset the drift from the reading. Because the seek angle routine on the Artemis is relative to it's initial zero (which was now drifted) I set the gyro reading to zero before seeking an angle. This makes the angle relative to the current position. Once the drift is accounted for, I once again reset the gyro and then rotate to the target angle.

```python
# Offset the gyro drift by rotating 20 degrees
ble.send_command(CMD.SET_GYRO, "0")
ble.send_command(CMD.SET_TARGET, "-20")
ble.send_command(CMD.SET_PID_GAINS,"ROTATION:|0.03:|0.0004:|0.0")
ble.send_command(CMD.SET_MODE, str(int(MODE_TYPE.SEEK_ANGLE)))
await asyncio.sleep(3)
# Rotate to the target angle
ble.send_command(CMD.SET_GYRO, "0")
ble.send_command(CMD.SET_TARGET, str(angle))
ble.send_command(CMD.SET_PID_GAINS,"ROTATION:|0.03:|0.0004:|0.0")
ble.send_command(CMD.SET_MODE, str(int(MODE_TYPE.SEEK_ANGLE)))
await asyncio.sleep(3)
```

#### Move Forward for Time Proportional to Distance
Because I'm using an iterative approach it doesn't matter if the car undershoots the target, as long as it gets closer on each iteration. This also removes the need for measuring the distance traveled, which can be difficult with the given sensors. I multiply the distance by an experimentally determined constant to approximate traveling the correct distance, and allow the Artemis to take care of the exact timing and PID control for driving straight . 

```python
# Move forward proportional to the distance to the waypoint
ble.send_command(CMD.SET_PID_GAINS,"ROTATION:|0.03:|0.0004:|0.0")
t = dist * 2000
print("Moving forward for: " + str(t))
ble.send_command(CMD.SET_TARGET, str(t))
ble.send_command(CMD.SET_MODE, str(int(MODE_TYPE.MOVE_FORWARD)))
await asyncio.sleep(t / 1000.0 + 1)
ble.send_command(CMD.SET_PID_GAINS,"ROTATION:|0.03:|0.0004:|0.0")
ble.send_command(CMD.SET_MODE, str(int(MODE_TYPE.BRAKE)))
await asyncio.sleep(0.5)
```
#### The Full Code

Here is the algorithm in full:

```python
# Reset Plots
cmdr.reset_plotter()

# Init Uniform Belief
loc.init_grid_beliefs()

# Store all of the waypoints
points = np.array([[-4, -3], [-2, -1], [1, -1], [2, -3], [5, -3], [5, -2], [5, 3], [0, 3], [0, 0]])

points = points * 0.3048 # Convert from feet to meters

target_index = 1

while True:
    # Get Observation Data by executing a 360 degree rotation motion
    await loc.get_observation_data()

    # Run Update Step
    loc.update_step()
    loc.plot_update_step_data(plot_data=True)
    
    # Locate the robot and its target
    target = points[target_index, :]
    pose_est = np.array(loc.get_current_estimate())
    print("Guess at pose: " + str(pose_est))
    print("Target position: " + str(target))
    print("Estimated position: " + str(pose_est[0:2]))
    
    # Check if the robot is on its target
    if (np.abs(target - pose_est[0:2]) < 0.3).all():
        print("Hit waypoint " + str(target))
        target_index+=1
        target = points[target_index, :]
        print("New target: " + str(target))
        
    # Find the distance and angle to target
    delta = target - pose_est[0:2]
    dist = np.linalg.norm(delta)
    print("Angle estimate: " + str(pose_est[2]))
    angle = np.rad2deg(np.arctan2(delta[1], delta[0])) - pose_est[2]
    
    print("Angle to target: " + str(angle))
    print("Distance to target: " + str(dist))
    
    # Offset the gyro drift by rotating 20 degrees
    ble.send_command(CMD.SET_GYRO, "0")
    ble.send_command(CMD.SET_TARGET, "-20")
    ble.send_command(CMD.SET_PID_GAINS,"ROTATION:|0.03:|0.0004:|0.0")
    ble.send_command(CMD.SET_MODE, str(int(MODE_TYPE.SEEK_ANGLE)))
    await asyncio.sleep(3)
    # Rotate to the target angle
    ble.send_command(CMD.SET_GYRO, "0")
    ble.send_command(CMD.SET_TARGET, str(angle))
    ble.send_command(CMD.SET_PID_GAINS,"ROTATION:|0.03:|0.0004:|0.0")
    ble.send_command(CMD.SET_MODE, str(int(MODE_TYPE.SEEK_ANGLE)))
    await asyncio.sleep(3)
    # Move forward proportional to the distance to the waypoint
    ble.send_command(CMD.SET_PID_GAINS,"ROTATION:|0.03:|0.0004:|0.0")
    t = dist * 2000
    print("Moving forward for: " + str(t))
    ble.send_command(CMD.SET_TARGET, str(t))
    ble.send_command(CMD.SET_MODE, str(int(MODE_TYPE.MOVE_FORWARD)))
    await asyncio.sleep(t / 1000.0 + 1)
    ble.send_command(CMD.SET_PID_GAINS,"ROTATION:|0.03:|0.0004:|0.0")
    ble.send_command(CMD.SET_MODE, str(int(MODE_TYPE.BRAKE)))
    await asyncio.sleep(0.5)
        
    
```

And here is a sample output from one step of the algorithm.

```txt
2023-05-09 10:39:22,638 | INFO     |: Update Step
2023-05-09 10:39:22,759 | INFO     |:      | Update Time: 0.119 secs
2023-05-09 10:39:22,762 | INFO     |: Bel index     : (14, 12, 34) with prob = 0.4493254
2023-05-09 10:39:22,763 | INFO     |: Bel_bar prob at index = 3.2150205761316875e-05
2023-05-09 10:39:22,765 | INFO     |: Belief        : (0.533, 0.533, -7.500)
Guess at pose: [ 0.5334  0.5334 -7.5   ]
Target position: [0.7 1.3]
Estimated position: [0.5334 0.5334]
Angle estimate: -7.5
Angle to target: 80.2389498399404
Distance to target: 0.7844941809854292
Moving forward for: 1568.9883619708585
```

## Results

I was unfortunately unable to finish this lab. The robot was able to successfully navigate two waypoints, then the motor drivers would overheat and cause the movement to become erratic. I hypothesize this had something to do with the rapid direction changes from the PID loop for angular velocity control. My laptop also decided to kick the bucket while debugging this issue, so thats all she wrote folks! I fully believe this approach would have been effective if I had time to debug the hardware, but alas. 

Thanks for a great semester to everyone involved. Live laugh fast robots.