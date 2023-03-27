## Introduction

A key issue with the car up to this point is the speed of the control loop. The loop is limited by when it receives new data, which is the sample time of the TOF sensor. Because TOF sensors need multiple sames to get an accurate result, the sample time is around 150 ms, or 6.66 Hz. A good control loop aims to be in the kHz, so we have a problem. A Kalman filter is one way of attacking this problem.

The Kalman filter uses a model of the robots dynamics, its control inputs (what the program tells the robot to do), and estimations of the error in the system to generate a probability distribution over the location of the robot. That is to say, it predicts what the robot will do and gives a measure of its certainty. We can use this model to predict our TOF measurements even when we don't have a new measurements. This allows us to speed up the control loop by orders of magnitude.

## Establishing a Model

To use the Kalman filter we first need a model of the dynamics of the robot. Specifically, we want to know how the sensor readings will react to a robot driving straight at a wall.


## Testing on Old Data

## Implementing on the Artemis

## Results