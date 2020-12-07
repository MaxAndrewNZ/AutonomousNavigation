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
    - Eigen 3.3.8
    - ORB_SLAM2

Think I need root access to install ROS. Will likely need ROS.

Tried running the RGB-D example using data downloaded from http://vision.in.tum.de/data/datasets/rgbd-dataset/download. Ran into issues with associations. Therefore, was not able to run the examples.

Leaving ORB-SLAM2 for now. Will attempt multi sensor setups.

## Determining Logic Behind Two RGB-D Camera Setup - Dec 1
This setup will have two D435 cameras mounted to the rover. One on the front of the vehicle, and one on the following side. This should provide a similar result to the existing methods using three ultrasonic sensors.

- The camera mounted to the front will be used for both obstacle avoidance and row following. 
- The side mounted camera will only be used for row following.

### States

- The vehicle is at the target distance from the row wall. Therefore, remain navigating forwards.
- The vehicle is not at the target distance. Turn vehicle to reach target distance.
- The angle of the vehicle relative to the wall is not 0. Turn the vehicle to make the angle 0. Will need to determine if the change in angle is due to reaching the end of the row. The forwards facing camera can be used to determine this.
- If the end of the row is reached, turn at a constant distance around the end.

### Attempting to adjust Landrov to have camera facing wall

The initial attempt to test a multisensor setup's viability is to use the existing landrov platform. Turning the camera to the side will help to test how well the system may work for wall following.

- The camera was turned on the Landrov. It now faces directly to the left of the vehicles navigation direction.
- The `side_camera_test` holds the python scripts for this test.
- Exiting files from my full year project were used with major adjustments.
- This system is very rough, using only two points of detection. Similar to existing solutions with two ultrasonic sensors on the side of the vehicle.
- The solution will be tested on the Landrov, inside.

#### Test 1 - Landrov Inside

The rover followed the walls, measuring distance from them accurately. However, the turning was very jumpy and aggressive. Leading to the vehicle moving to close to the wall, then too far away and repeating that pattern.

Once the system is improved to use all information. The angle should be used to create a smooth turning system. This should remove the jumpiness and overshooting of the current system.

## Optimising Landrov - Dec 2

- Attempting to move the pointcloud downsampling to the Landrov. This should hopefully reduce lag due to sending large pointcloud strings over wifi. Have not completed this due to having to install Open3D on the Landrov and likely having to do a lot of processing onboard the vehicle. I will revisit this point if it is necessary. For now it will return to the backlog.

- The Landrov was updated to no longer send rgb and depth images. This should increase its efficiency when wending pointclouds over wifi.

## Set Turning Times - Dec 2

- Creating methods to handle turning the vehicle a set angle.
- Have to calculate the angle of the vehicle relative to the wall. This could be done with plane estimation or with a simplier two point calculation.

### Testing Turning - Dec 2

- The system could follow a flat wall, making small angle adjustments when needed. These adjustments were quite aggressive.
- The current method resulted in issues wen the rover was outside of the target distance error range. It would turn towards the wall if it was too far away. However, this resuted in the average distance increasing due to taking the two region average for the distance average. This will need to change to fix this issue. I will investigate a more robust method for wall following using a depth camera.

## Lead and Lag Regions - Dec 2

Attempting to implement a method that breaks up the pointcloud into two regions. These are the leading and lagging regions. This method will not currently handle obstacles infront of the vehicle.
This will be resolved when the second camera is mounted to the front of the vehicle.

### State Concepts

- if lag and lead are far then cant find wall. Stop or search for wall.
- if lag close and lead far then at end of row. Complete an end of row turn.
- if lead close then following row or starting on the row. Use plane estimation to adjust robots angle and distance from wall.

### Turning Issues

This method will likely suffer from the aggresive turning issues from previous methods. Need to speed up the processing so updates can be made more frequently. This may help to smooth the vehicles navigation.

### Testing Method

## 45 degree single camera - Dec 3

This may be a suitable approach.
Thinking of splitting up the cameras vision space into a number of wedges. Then acting according to the data collected by these.

### Linear Velocities

The method used 4 different velocities for the linear velocity component.

- 0 - Minimal distance is less that the target distance
- 0.5 - Wall distance is less than 2 times the target distance
- 0.4 - The vehicles angle offset from the wall is greater than 1.75 radians
- 1 - For faster movement towards the wall

### Distance Error

Difference between target distance and the shortest measured distance.

Proportional derivative (PD) controller used to fix issues of error.

- Proportional constant - currently set to 15
- Derivative constant - currently set to 0

### Angle Error

- The minimum distance's angle from the wall must be pi/2 for left following.
- The angular velocity is the sum of interventions from PD distance control and P angle control

The angular velocity is the sum of interventions from PD distance control and P angle control

### Testing 45 degree 5 region

Not expecting this method to work at this stage.

## 45 degree single camera with point clouds - Dec 4

- Have decided to attempt the 45 degree setup as discussed before, but using a pointcloud instead of a depth map.
- This should avoid the issue with finding an angle to a point on a depth map.
- However, this method has an issue. Finding the closest point to the camera in the pointcloud is very comutationally expensive.
- Flattened the pointcloud to ignore y-axis.
- Added a k nearest neighbour method to calculate the closest point to the vehicle.

### Testing 45 degree pointcloud method - Dec 7

- The solution did not perform well.
- Its turning and speed control almost felt random.
- Am looking into starting with a more simplified model.

## Simplified Method - Dec 7

- The target distance is ignored for now.
- Focusing on aligning robot with the wall using the closest point method

### Test

- The robot was tested just with aligning to the nearest point. This worked ok. However, the robot would often overshoot the target. This could likely be fixed through sending pointclouds at a faster rate. The method currently takes 0.6 seconds to process each pointcloud.
- Linear velocity was added to the robot. It followed some walls, however, the performance remained quite poor.
- I would like to test this in a larger area to determine if the method has any promise. However, for now, it is not looking too effective.
- The aggression of the method could be eased through smoothing the pointcloud. This may be achieved through voxel downsampling, agressive outlier removal, or possibly the use of convex hulls.

## Simple 2D line follower - Dec 8

- Attempting to implement a following method for one side of the vineyard row. 
- The camera is facing forwards
- A line is fitted to the data. 
- The equation of this line is used to calculate the angular and linear error,
- These errors are then converted into motor commands.

## Test

- 