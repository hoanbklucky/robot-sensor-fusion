# Worked examples

## Before you calculate

The **state** contains what you want to estimate. A **measurement** is what a sensor reports. The **covariance** describes uncertainty and relationships between errors. A variance is a squared standard deviation. For example, a GPS position standard deviation of 0.5 m corresponds to a measurement variance of 0.25 m².

Use a superscript minus for a prediction and plus for a corrected estimate. In a linear model, the prediction is x⁻ = Fx + Bu and P⁻ = FPFᵀ + Q. Here Q represents uncertainty added by the motion model. The update below assumes independent motion and measurement noise and uses a single scalar observation at a time.

| Step | Equation | Meaning |
|---|---|---|
| Predict measurement | ẑ = Hx⁻ | Convert the estimated state into the sensor’s units |
| Innovation | y = z − ẑ | Calculate the measurement surprise |
| Innovation variance | S = HP⁻Hᵀ + R | Combine projected state uncertainty with sensor noise |
| Gain | K = P⁻Hᵀ / S | Decide how much each state component should respond |
| Correct | x⁺ = x⁻ + Ky | Move the estimate by the gain times the surprise |
| Update covariance | P⁺ = P⁻ − K(HP⁻) | Reduce uncertainty along observed directions in this scalar update |

In the EKF, use the nonlinear sensor function h(x⁻) to calculate ẑ and its Jacobian H to calculate S and K. The worked EKF isolates measurement linearization. For a nonlinear motion model, evaluate the motion Jacobian at the previous corrected state for covariance propagation.

## 1. Fuse wheel odometry and a position fix

At the previous tick, the robot’s estimated position is 9 m with variance 0.8 m². Wheel odometry reports a displacement of 1 m and adds Q = 0.2 m². A position sensor now reports 10.6 m with R = 0.25 m².

**Predict first.** Where is the robot according to odometry? Should the corrected position be closer to the odometry estimate or to the sensor reading?

<details>
<summary>Reveal the calculation</summary>

1. x⁻ = 9 + 1 = **10 m** and P⁻ = 0.8 + 0.2 = **1 m²**.
2. H = 1, so the predicted measurement is 10 m and y = 10.6 − 10 = **0.6 m**.
3. S = 1 + 0.25 = **1.25 m²** and K = 1/1.25 = **0.8**.
4. x⁺ = 10 + 0.8 × 0.6 = **10.48 m** and P⁺ = (1 − 0.8) × 1 = **0.20 m²**.

With R = 4 m², K falls to 0.2 and the same reading gives **10.12 m**. With no reading, keep the prediction at **10 m** and retain **1 m²** variance.

</details>

## 2. Why a position fix changes velocity

A robot starts with [position, velocity] = [0 m, 1 m/s], covariance diag(1, 1) in the corresponding squared units, and a one-second time step. Use a constant-velocity model with F = [[1, 1], [0, 1]]. Set Q = 0 for this single hand calculation. The position sensor reports 1.6 m with R = 1 m².

**Predict first.** The sensor measures position only. Will the estimated velocity change?

<details>
<summary>Reveal the calculation</summary>

1. x⁻ = **[1, 1]** and P⁻ = FPFᵀ = **[[2, 1], [1, 1]]**. The off-diagonal covariance is positive because a higher velocity error also makes the position prediction higher.
2. H = [1, 0], so y = 1.6 − 1 = **0.6 m** and S = 2 + 1 = **3 m²**.
3. K = **[2/3, 1/3]ᵀ**. The second entry comes from the cross-covariance and has units s⁻¹.
4. x⁺ = [1, 1] + [2/3, 1/3] × 0.6 = **[1.4 m, 1.2 m/s]**.
5. P⁺ = **[[2/3, 1/3], [1/3, 2/3]]**.

If the predicted covariance were diagonal, the velocity gain would be zero. The robot can infer something about velocity because the motion model connects velocity to later position.

</details>

## 3. EKF update from a beacon range

A dead-reckoning prediction places the robot at (3, 4) m with P⁻ = I m². A beacon at (0, 0) reports range 6 m with R = 0.25 m². The observation is h(x, y) = √(x² + y²). The simulator starts from the supplied prediction.

**Predict first.** Does the range make the robot move toward or away from the beacon? What part of position does a single range tell us least about?

<details>
<summary>Reveal the calculation</summary>

1. ẑ = √(3² + 4²) = **5 m**. The innovation is **+1 m**.
2. Differentiate h with respect to x and y. Each derivative is the corresponding coordinate divided by range. Evaluate at (3, 4) to get H = **[0.6, 0.8]**.
3. S = 0.6² + 0.8² + 0.25 = **1.25 m²**. K = **[0.48, 0.64]ᵀ**.
4. Correction = K × 1 m. The result is **(3.48, 4.64) m**, at range 5.8 m.
5. P⁺ = I − KH = **[[0.712, −0.384], [−0.384, 0.488]] m²**. Variance along the radial direction becomes 0.2 m². The tangent variance stays at 1 m² in this linearized update.

All points on the 6 m circle have the same range. The prior estimate supplies the direction needed to pick a local correction. The Jacobian is undefined at the beacon, and a local linear approximation can be poor when uncertainty is large relative to range.

</details>

## 50-minute classroom activity

| Time | Task |
|---|---|
| 0–10 min | Work the scalar example. Change R and explain why the same innovation gives a different correction. |
| 10–23 min | Pair up and compute P⁻ for the position–velocity example. Circle the number that lets position change velocity. |
| 23–38 min | Sketch the circle, compute the range Jacobian by partial derivatives, and check the EKF correction in the simulator. |
| 38–47 min | Use repeated updates. Compare a low and high R on the same data, then disable all fixes and watch the variance. |
| 47–50 min | Exit ticket. A range is 0.5 m *shorter* than expected in example 3. Predict the sign of both coordinate corrections and explain why. |

<details>
<summary>Exit ticket answer</summary>

Both corrections are negative because the innovation is −0.5 m and both gain entries are positive. The corrected position is (2.76, 3.68) m.

</details>

### From the classroom to a mobile robot

In this 1D lab, wheel odometry supplies the motion increment and an absolute position sensor supplies a correction. A planar robot can instead estimate [x, y, heading] using a motion model driven by wheel speeds and yaw rate. GPS can correct horizontal position after transformation to a local Cartesian frame. A landmark sensor can contribute range or bearing. Treat a missing measurement as prediction-only, wrap bearing innovations at ±π, and avoid treating two estimates derived from the same raw sensor as independent observations. Record the measurement timestamp and use the elapsed time between events.
