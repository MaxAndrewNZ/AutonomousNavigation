# Documented Progress

## One vs Two Cameras - 1 Dec

### One Camera

#### Pros and Cons

    - Lower cost
    - Will need to use past information to determine the current state.
    - May have issues at start of vineyard row.
    - Need to assess its viability for turning at end of row.
    - The accuracy of this method may not be appropriate.

#### Existing Implementations

    - SLAM (Simultaneous localisation and mapping) could use Rtabmap or imu_filter_madwich algorithms. These are ROS packages. The D435 does not have a built in IMU. Therefore, the change in location must be determined another way. This could be achieved through aligning pointclouds with an ICP algorthm or using pioneer's wheel odometry.
    - ORB-SLAM2


### Two Cameras

#### Pros and Cons

    - Higher cost.
    - May lead to a more complex solution if using information from both cameras for navigation.
    - Should lead to more accurate row following.
    - The rover can only follow one side of the plant row. However, this was already the case due to the scanning rig viewing one side of the plant row. Therefore, this should not be an issue.
    - Should allow for simple end of row turning.
    - Has no issues with start of row navigation.

#### Existing Implementations
    - Fuzzy logic system using 7 ultrasonic sensors at 15 degree intervals.
    - Fuzzy with three ultrasonic sensors, two on one side of vehicle. at front and back. Third sensor facing forward in th edirection of movement. This system could be translated to the vineyard following application. The D435 could remain on the front and used for obstacle avoidance. Two ultrasonic sensors could be mounted to the side of the vehicle to be used for row following. 
![Three Sensor Setup](/images/three-sensor.jpeg)
    
    - Could use fuzzy neural network to smooth path. Found recent paper that led to only a small improvement using this method.

## Attempting Using ORB-SLAM2 - 1 Dec

Hoping to set up ORB-SLAM2 for the single D435 camera. This will allow the scene to be mapped as the rover moves down the row. This data can hopefully be used to determine the rovers distance from the plant row. Therefore, leading to the ability to adjust its distance to be at the target distance from the row.

### Installing

    - Pangolin
    - OpenCV 4.5.0
    - eigen 3.3.8
    - ORB_SLAM2

Think I need root access to install ROS. Will likely need ROS.

Tried running the RGB-D example using data downloaded from http://vision.in.tum.de/data/datasets/rgbd-dataset/download. Ran into issues with associations. Therfore, was not able to run the examples.

Leaving ORB-SLAM2 for now. Will attempt multi sensor setups.

## Determining Logic Behind Two RGB-D Camera Setup - Dec 1
This setup will have two D435 cameras mounted to the rover. One on the front of the vehicle, and one on the following side. This should provide a similar result to the existing methods using three ultrasonic sensors.

- The camera mounted to the front will be used for both obstacle avoidance and row following. 
- The side mounted camera will only be used for row following.