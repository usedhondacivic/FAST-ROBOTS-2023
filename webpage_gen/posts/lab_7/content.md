## Introduction

A key issue with the car up to this point is the speed of the control loop. The loop is limited by when it receives new data, which is the sample time of the TOF sensor. Because TOF sensors need multiple sames to get an accurate result, the sample time is around 150 ms, or 6.66 Hz. A good control loop aims to be in the kHz, so we have a problem. A Kalman filter is one way of attacking this problem.

The Kalman filter uses a model of the robots dynamics, its control inputs (what the program tells the robot to do), and estimations of the error in the system to generate a probability distribution over the location of the robot. That is to say, it predicts what the robot will do and gives a measure of its certainty. We can use this model to predict our TOF measurements even when we don't have a new measurements. This allows us to speed up the control loop by orders of magnitude.

For help on this lab, I referenced Anya Prabowo's fantastic lab report, which you can find here: [https://anyafp.github.io/ece4960/labs/lab7/](https://anyafp.github.io/ece4960/labs/lab7/)

## Establishing a Model

To use the Kalman filter we first need a model of the dynamics of the robot. As derived in class, we know that the state space equation for the car is

```latex
\begin{bmatrix}
a & b \\
c & d 
\end{bmatrix}
```

I used 45% duty cycle (115 / 255) for my tests because it was the average PWM output during my stunt for lab 6. I set the car to output this to the wheels, and ran it straight into a wall.

Here is the car's TOF readings over time. You can see the point it hits the wall, after which the data isn't useful for finding our constants. I cropped the data to only the relevant range, then calculated its derivative to get the velocity of the car.

![The original data](./assets/orig_data.png)

![The clipped data](./assets/clipped_tof_data.png)

![The velocity data](./assets/clipped_tof_average_data.png)

There was a box to the right side of the car on this run, resulting in a spike on the sensor B readings. Because of this, I use only sensor A's readings. As is shown in the velocity graph, the car did not reach steady state, which would result in a constant velocity.

Because I used a high speed for my car in lab 6, I was unable to get the car to reach steady state in the hallway. To find the rise time and steady state speed, I instead fitted a polynomial curve to the data and used that to find my constants. Below is the curve fitted to the TOF data + the velocity from this data.

![The fitted TOF data](./assets/tof_fitted.png)

![Velocity from the fitted TOF data](./assets/velocity_fitted.png)

We can see that at 45% duty cycle


## Testing on Old Data

## Implementing on the Artemis

## Results