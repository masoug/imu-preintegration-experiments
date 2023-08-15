# GTSAM IMU Preintegration Experiments
Test programs to understand how GTSAM's imu preintegration algorithm works.

# Does the preintegrator correct roll+pitch using gravity?
According to the test from `gravity.py`, the answer is no. Here's the output:
```
init state (level):
R: [
        1, 0, 0;
        0, 1, 0;
        0, 0, 1
]
p: 0 0 0
v: 0 0 0


predicted state (level):
R: [
        1, 0, 0;
        0, 1, 0;
        0, 0, 1
]
p: 0 0 0
v: 0 0 0


factor error: [0. 0. 0. 0. 0. 0. 0. 0. 0.]

init state (pitched):
R: [
        6.12323e-17, 0, -1;
        0, 1, 0;
        1, 0, 6.12323e-17
]
p: 0 0 0
v: 0 0 0


predicted state (pitched):
R: [
        6.12323e-17, 0, -1;
        0, 1, 0;
        1, 0, 6.12323e-17
]
p: -4.905      0 -4.905
v: -9.81     0 -9.81


factor error: [0. 0. 0. 0. 0. 0. 0. 0. 0.]
```
This tests evaluates the `ImuFactor` error across two scenarios:
  1. A stationary, "level" configuration with zero velocity. Since there's no acceleration (outside of gravity) there should be no motion and the factor error should be zero too.
  2. A "pitched" configuration where the initial pose is situated at the origin facing the sky. The predicted pose will then be down and behind the initial pose due to both the gravity measured by the accelerometer and the PIM gravity correction. The factor error in this case, is also zero.

Scenario 2 serves as a counter example to the assertion that the `PreintegratedImuMeasurements` and `ImuFactor` classes can correct pitch (and roll) error, given that two distinctly different state configurations yield zero error. In other words, since the optimizer can not distinguish between the two configurations because they are both appear perfectly optimal, the optimizer will not attempt to correct the attitudes of the pose variables in scenario 2. In conclusion, the preintegrated IMU factors can not correct roll and pitch error using the gravity vector. An additional factor such as the [`Pose3AttitudeFactor`](https://github.com/borglab/gtsam/blob/4.1/gtsam/navigation/AttitudeFactor.h#L152) should be used to apply roll and pitch corrections.