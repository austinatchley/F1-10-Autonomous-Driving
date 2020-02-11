# Checkpoint 1.2 Progress Report

Logan Zartman

Austin Atchley

## 1. Mathematical computation of our 1-D time optimal controller:

## 2. Code organization:
Our code is largely organized the same way as checkpoint 1.1, but we did move some functionality to helper functions. Most of the new code is in the form of mathematical calculationsthat run each frame, called from `Navigation::Run()`. We perform the calculations described above according to the slides from class, but we did some measurement of our own to ensure that the latency values were optimal. We use the "Strategy 2" approach to latency compensation, meaning that we assume the car will continue to accelerate at the same rate as its last commanded acceleration. We predict the position and velocity using these values.

Instead of keeping track of odometer-measured speed and what we called `toc_speed` before, we collect the odometry measurements and predict where the car will be during the control execution and motor actuation stages. We use these values to improve our accuracy on `time_to_stop` and to choose the correct acceleration value.

As latency compensation was the main objective on this checkpoint, that sums up most of the relevant work we have done so far.

## 3. Parameter tuning:

## 4. Challenges faced:
We had trouble understanding where/when to compensate for latency. We implemented an initial logical approach and debugged it from there to tune the values and where we should use each position/speed value.

These challenges weren't major roadblocks though. We worked through them without being blocked for too long.

## 5. Contributions of each team member:

Austin - Initial approach and structure for latency compensation. Designated car driver

Logan - Fine-tuning for latency compensation implementation and other parameters. Visualization and collection of data points

## 6. GitHub Repo
[https://github.com/austinatchley/F1-10-Autonomous-Driving](https://github.com/austinatchley/F1-10-Autonomous-Driving)

## 7. Video Demos
[https://drive.google.com/file/d/1-6I8ubXC_vUiwEjwKUGETv0ijHdRV1WZ/view?usp=sharing](2 meters distance on pre-measured tape with default velocity/acceleration)
[https://drive.google.com/file/d/17qz0Eb2-Sf8woBZr_DeGcpqaZx4tWImE/view?usp=sharing](2 meters distance with curvature of -1)
[https://drive.google.com/file/d/1vfNl40W3i84cjVTiD2gXZRp0lGstgpvQ/view?usp=sharing](0.5 meters)

## 8. Future improvements:
Sometimes the car lurches at the end because it hasn't quite reached the desired distance, and it ends up passing over the mark slightly. It is still within the margin of error, but we would like to make stopping as smooth as possible.

Our code structure isn't exactly robust yet, but we don't want to add too much abstraction until we need it. In the future checkpoints, we will improve this aspect of our project.
