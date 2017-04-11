# ALineD: Algorithms for Line-Based Data Correspondences

This library is based on the paper "Pose Estimation from Line Correspondence using Direct Linear Transform" by Pribyl,B. 2016,
the R_and_T algorithm as shown in "Robust Methods for Estimating Pose and a Sensitivity Analysis" by Kumar,R.,Hanson,A.R.,1994,
loss functions as explained in "Bundle Adjustment - A modern Synthesis", by Triggs,B., 1999,
and the paper "A Brief Description of the Levenberg-Marquardt Algorithm Implemened by levmar", by Manolis,I., Lourakis, A., 2005.
It consists of a fully line-based DLT algorithm as described in the above paper by Pribyl, a modification of the R_and_T algorithm
and an Extended Kalman Filter for sensor fusion. The Kalman Filter at the moment contains a constant velocity model, but is customizeable.
It contains elements discussed in "A Primer on the Differential Calculus of 3D Orientations", by Bloesch et al., 2016, "Practical
Parametrization of Rotations Using the Exponential Map  ", by Grassia, F., 1998 and "Quaternion kinematics for the error-state KF",
by Solà, J., 2016.

![alt text](https://github.com/AndreaLampart/alined/blob/master/img/alined.png "Pose from lines")


## Motivation

Modern SLAM systems use point feature correspondences to extract camera motion. Points are limiting in a way that they
contain less structural information about the environment. Not many fully line-based systems have been published.
This library should be a help to whoever will tackle the problem of line-based SLAM or VO.
Please be aware that this software is still under development and it might or might not be discontinued.
No warranty for further development is given.

## Installation

The software is self-contained and uses no additional dependencies except Eigen3. First, you should install the Eigen library by following their steps provided on the official website. Afterwards just clone the repository:

```bash
git clone directory-name.git
```


## How to use

The library can be included into any project and can be run from within a ros node. The following will calculate the camera pose:


```c++
// Configuration is sent via bitmasking
Alined alined(AL_COMBINED_LINES|AL_USE_REFINE|AL_LEVENBERG_MARQUARDT|AL_HUBER_LOSS);
alined.setLossScale(1.0);

// World Line endpoints (Don't need to be exact)
Eigen::Matrix<double,4,Eigen::Dynamic> X_w;

// Projected Line endpoints in normalized camera space
EIgen::Matrix<double,3,Eigen::Dynamic> x_c;

// Line-Based DLT
Eigen::Matrix4d pose = alined.poseFromLines(x_c,X_w);

// Iterative Approach
Eigen::Matrix4d tf = alined.poseFromLinesIterative(pose, x_c, X_w);
```

The Kalman Filter is used as follows:



```c++
// Set initial state
State initial_state;
initial_state.position() = Eigen::Vector3d(0,0,0);
initial_state.velocity() = Eigen::Vector3d(0.0,0.0,0.0);
initial_state.rotation() = Eigen::Quaterniond::Identity();
initial_state.ang_vel()  = Eigen::Vector3d(0.0,0.0,0.0);

// Set noise variances
Eigen::Matrix<double,13,1> noise_mm;
noise_mm << 0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1;

MotionModel* mm = new MotionModel();
SensorModel* sm = new SensorModel();
KalmanFilter kalman_filter(KF_CONSTANT_VELOCITY | KF_USE_QUATERNIONS);

kalman_filter.setMotionModel(mm);
kalman_filter.pushSetSensorModel(sm);
kalman_filter.setCovarianceMMByVector(noise_mm);
kalman_filter.setNoiseVariances(noise_mm);
kalman_filter.setInitialState(initial_state);

// Print out initial state
kalman_filter.printState();

// At update:
State update_state;
update_state.position() = Eigen::Vector3d(0,0,0);
update_state.velocity() = Eigen::Vector3d(1e-9,1e-9,1e-9);
update_state.rotation() = Eigen::Quaterniond::Identity();
update_state.ang_vel()  = Eigen::Vector3d(0.0,0.0,0.0);

int64_t time_since_last_update_in_ns = 1000000;

kalman_filter.predict(time_since_last_update_in_ns);
kalman_filter.update(KF_SENSOR_1, update_state);

// Print updated state
kalman_filter.printState();
kalman_filter.printCovariance();

// shutdown
kalman_filter.exit();
```


## Contributors

If you want to contribute to this library, feel free to send your contributor request to the mail address provided in the package.xml.

## License

 ALineD is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 ALineD is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with ALineD. If not, see <http://www.gnu.org/licenses/>.

