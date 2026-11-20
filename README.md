# Robot Sensor Fusion

Learn to combine motion predictions and sensor measurements through worked Kalman filter (KF) and extended Kalman filter (EKF) examples for mobile robots.

## Start here

1. Read the [worked examples](worked-examples.md). Calculate the innovation and predict which way the estimate should move before revealing the answer.
2. Download the repository with **Code → Download ZIP**, extract it, and open **index.html** in a browser. The simulator runs offline without Python, ROS, or a physical robot.
3. Use **Next step** to reveal the prediction, innovation, gain, and correction. Change the sensor variance or drop the measurement and predict the result before stepping through again.
4. Try the 50-minute classroom activity in the [worked examples](worked-examples.md#50-minute-classroom-activity).

| Lesson | Robot scenario | What you will see |
|---|---|---|
| Scalar KF | Fuse wheel odometry with a position fix | A reliable sensor pulls the estimate farther toward its reading |
| Position and velocity KF | Correct a moving robot using a position fix | The cross-covariance allows a position measurement to correct velocity |
| Landmark-range EKF | Locate a robot in 2D using a range to a beacon | The measurement Jacobian determines the direction of correction |
| Repeated updates | Track a robot with intermittent position fixes | Drift during a dropout and correction when measurements return |

The simulator starts from deterministic teaching examples. For actual robot integration, align timestamps and coordinate frames and characterize the noise from recorded data.

[Further reading — Welch and Bishop, An Introduction to the Kalman Filter](https://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/WELCH/kalman.html)
