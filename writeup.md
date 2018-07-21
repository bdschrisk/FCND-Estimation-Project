## Project: Building an Estimator
![Estimation](/images/mag-good-solution.png)

---

## Rubric Points
### Here I will consider the [rubric](https://review.udacity.com/#!/rubrics/1807/view) points individually and describe how I addressed each point in my implementation.

---

### Explain the Implementation

#### 1. Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.
To determine the standard deviations of the sensor measurement noises of both the GLONASS and Accelerometer sensors, a sensor log dump was analysed using Pandas and an average of the standard deviations was used.

__Accelerometer sensor noise__
```
>>> accel.describe()

              time   Quad.Est.E.Yaw
count  2079.000000      2079.000000
mean      5.199917         0.004818
std       3.001505         0.035651
min       0.005000        -0.107478
25%       2.602494        -0.015131
50%       5.199806         0.004087
75%       7.797117         0.026474
max      10.395571         0.141131

>>> accel_std = 0.035651
```
Unfortunately the standard deviation of the Accelerometer **0.035651** from the dataset did not perform to specifications and required manual tuning.  After some quick experimentation a new value of **0.52051** proved worthy.
```
>>> gps.describe()
             time   Quad1.PosFollowErr   Quad2.PosFollowErr   Quad3.PosFollowErr
count  370.000000           370.000000           370.000000           370.000000  
mean     0.927503             0.793700             0.594827             0.631693
std      0.534778             0.696695             0.749921             0.718603
min      0.005000             0.019484             0.001246             0.088746
25%      0.466248             0.244893             0.042231             0.114477
50%      0.927492             0.454691             0.105009             0.154589
75%      1.388759             1.499786             1.251424             1.236974
max      1.850030             2.000000             2.000000             2.000000

>>> gps_std = np.mean([0.696695, 0.749921, 0.718603])
>>> gps_std

0.721739
```
Using the approximated standard deviation from the 3 different positional errors reduced the sensor noise to an acceptable level.

#### 2. Implement a better rate gyro attitude integration scheme in the UpdateFromIMU() function.
Here the provided Quarternion class was used to integrate the body rates to an acceptable attitude estimator of within 0.1 radians.  This was accomplished by using the following:
```
Quaternion<float> q = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
q.IntegrateBodyRate(gyro, dtIMU);
```
The yaw angle was then normalised using the provided helper function in the Angles include file
```
ekfState(6) = AngleNormF(ekfState(6));
```
As part of the final solution I added a moving average filter to provide greater stability in noisy measurements from the IMU.  This gave a small improvement which helped overcome small errors in the sensor measurements.
```
predictedPitch = ((1.0 - ALPHA) * q.Pitch()) + (ALPHA * (pitchEst + lastGyro.y * dtIMU));
predictedRoll = ((1.0 - ALPHA) * q.Roll()) + (ALPHA * (rollEst + lastGyro.x * dtIMU));
predictedYaw = ((1.0 - BETA) * q.Yaw()) + (BETA * (ekfState(6) + lastGyro.z * dtIMU));
```
*The moving average parameters ALPHA and BETA, were set to 0.014 and 0.0014, consecutively*

#### 3. Implement all of the elements of the prediction step for the estimator.
In the prediction step, the estimator uses the belief from the previous update step based on previous sensor measurements and a time delta. To infer the new state the new roll, pitch and yaw estimates are used along with a time delta using the standard Newtonian physics equation:
```
X(t+1) = X(t) + velocity * delta(t)
```
Once the state is predicted, the current belief is updated based on the change in euler angles in state space and the rotation matrix Rbg_prime - given the acceleration and change in time.  The G_prime matrix which is the derivativation of the positions with respect to X and the bodyrate is applied the rotation matrix Rbg_prime to update the current belief.

Then, the EKF covariance update looks like this:
```
EKF_Cov(t+1) = G_prime * EKF_Cov(t) * G_prime.T + Q
```

#### 4. Implement all of the elements of the prediction step for the estimator.
To update the covariance matrix with the new belief based on a new sensor reading, the normalised difference of the current yaw angle and the new magnetometer data is applied.

Since the derivative of the yaw angle is 1 we can set the H_prime vector to be like so:
```
 H_prime = [0, 0, 0, 0, 0, 1]
```
The update angle error normalisation is performed by checking the difference and conforming the angle to be between +pi and -pi.

The new Z and H update parameters are passed to the `Update` function to update the posterior belief.

#### 5. Implement all of the elements of the prediction step for the estimator.
Because the GPS sensor returns both velocity and position values, the readings can map directly to the state belief, where the state vector is `[x, y, z, vx, vy, vz]`.

The derivative is then the Identity matrix:
```
 H_prime =  [1, 0, 0, 0, 0, 0]
            [0, 1, 0, 0, 0, 0]
            [0, 0, 1, 0, 0, 0]
            [0, 0, 0, 1, 0, 0]
            [0, 0, 0, 0, 1, 0]
            [0, 0, 0, 0, 0, 1]
```

The new Z and H update parameters are passed to the `Update` function to update the posterior belief.

#### Update equations:
The update step uses the process covariance and model covariance matrices along with the new sensor readings to model the posterior belief like so:

```
S = H * ekfCov * H.T) + R;
K = ekfCov * H.T * S.inv;
ekfState = ekfState + K * (z - zFromX);
I = Identity
ekfCov = (I - K * H) * ekfCov;
```