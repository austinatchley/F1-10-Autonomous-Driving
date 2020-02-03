# F1/10 Autonomous Driving

## Description
Austin Atchley and Logan Zartman

The code in this repo is used in conjunction with [another repo](https://github.com/ut-amrl/f1tenth_course) provided for us by Professor Joydeep Biswas for the class CS 378: F1/10 Autonomous Driving. It is used to pilot an autonomous RC car using the ROS environment.

## Build
1. `make -j`

2. Run `roscore`

3. Run the simulation binary and websocket from [this repo](https://github.com/ut-amrl/cs378_starter).

2. Execute `./bin/navigation`

## Checkpoint 1.1 Progress Report
1. Mathematical computation of our 1-D time optimal controller:

2. Code organization:

Most of the code we wrote for this project resides in `navigation/navigation.h` and `navigation/navigation.cc`. We began by sending a basic `AckermannCurvatureDriveMsg` to the car. We proceeded by storing state every time the car's callbacks were called. We used the data to calculate the distance we had traveled, and implemented the dead reckoning approach. This was all done in the `Navigation::Run` method. Next, we began to use the odometry data to get a better idea of where the car is. This made use of the `Navigation::UpdateOdometry` callback. We used this data to implement the approach outlined on the slides in class.

We predict where the car will be on the next timestep. If the car is not at full velocity and far enough away from the target destination, we accelerate at maximum acceleration. If the car would be past the deceleration target, we decelerate at maximum deceleration. Thus, the car stays under its limits and satisfies the 1D time-optimal control problem.

We use basic kinematic equations to make these calculations. In particular, `x=vt+(1/2)at^2` was useful to us. We take the velocity we've calculated with this equation and send it to the car with `ros::Publisher::publish`.

3. Challenges faced:

The main challenge we faced was reconciling the speed we calculated with the speed we received from the odometer readings. We weren't exactly sure how to combine the two pieces of data at first, but we came up with a solution that seems to work well enough so that our car lands within +-0.05m over 2m in the simulation. We will continue to improve the calculations in the coming checkpoints.

4. Contributions of each team member:

Austin - Initial structure, dead reckoning, and external tooling setup for the rest of the semester

Logan - Kinematic calculations, using odometry data to track position and velocity

5. [https://github.com/austinatchley/F1-10-Autonomous-Driving](https://github.com/austinatchley/F1-10-Autonomous-Driving)

6. Video demos are located in the `videos` subdirectory

7. See above

8. Future improvements:

We would like to have `_toc_speed` and `speed` readings converge. That is to say, we want to take our two separate speeds and get more accurate results as a result of having more data. Right now, we aren't sure if our method is the most correct way of combining the two readings. We were thinking of doing something similar to a PID controller, but this would have to happen once every tick, so it can't be too resource intensive.

Obviously, we would like the car to be able to go in other directions. This is for the next checkpoint.
