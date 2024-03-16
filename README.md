# <div align="center">SLAM with Obstacle Avoidance</div>
![image](https://github.com/WinstonHChou/winter-2024-final-project-team-7/assets/68310078/0ba1c6cb-c9e0-4cf7-905a-f5f16e6bb2ca)
### <div align="center"> MAE 148 Final Project </div>
#### <div align="center"> Team 7 Winter 2024 </div>

<div align="center">
    <img src="images\ucsdrobocar-148-07.webp" width="800" height="600">
</div>

## Table of Contents
  <ol>
    <li><a href="#team-members">Team Members</a></li>
    <li><a href="#abstract">Abstract</a></li>
    <li><a href="#what-we-promised">What We Promised</a></li>
    <li><a href="#accomplishments">Accomplishments</a></li>
    <li><a href="#challenges">Challenges</a></li>
    <li><a href="#final-project-videos">Final Project Videos</a></li>
    <li><a href="#hardware">Hardware</a></li>
    <li><a href="#software">Software</a></li>
        <ul>
            <li><a href="#obstacle-avoidance">Obstacle Avoidance</a></li>
            <li><a href="#pedestrian-detection">Pedestrian Detection</a></li>
        </ul>
    <li><a href="#gantt-chart">Gantt Chart</a></li>
    <li><a href="#course-deliverables">Course Deliverables</a></li>
    <li><a href="#project-reproduction">Project Reproduction</a></li>
    <li><a href="#acknowledgements">Acknowledgements</a></li>
    <li><a href="#contacts">Contacts</a></li>
  </ol>

<hr>

## Team Members
Winston Chou - MAE Ctrls & Robotics (MC34) - Class of 2026 - [LinkedIn](https://www.linkedin.com/in/winston-wei-han-chou-a02214249/)

Amir Riahi - ECE - UPS Student

Rayyan Khalid - MAE Ctrls & Robotics (MC34) - Class of 2025
<hr>

## Abstract
The project's goal is to develop a robotic system capable of mapping a new enclosed environment, determining a path from a specified starting point to a desired destination while avoiding obstacles in its path. This involves integrating sensors for environmental perception, implementing mapping and localization algorithms, designing path planning and obstacle avoidance strategies, and creating a robust control system for the robot's navigation.

The robot utilizes the ROS2 Navigation 2 stack and integrating LiDAR for SLAM (Simultaneous Localization and Mapping) along with the OAK-D Lite depth point cloud camera for real-time obstacle avoidance.

<hr>

## What We Promised
### Must Have
* Integrate LiDAR sensor(s) into the ROS2 system. Utilize the ROS2 Navigation 2 stack to perform SLAM using LiDAR data.

### Nice to Have
* Integrate the OAK-D Lite depth camera, and Develop algorithms within ROS2 to process the point cloud data generated by OAK-D Lite for real-time obstacle detection. (ONLY Detection, for now)
* To move the robot from a given location A to a desired location B using the ROS2 Navigation 2 stack and integrated sensors. (Not implemented yet)
<hr>

## Accomplishments
* SLAM development accomplished
  * It enables the robot to map an unknown environment, and to locate its position. 
* Obstacle Avoidance
  * Utilize its depth sensing capabilities to generate a point cloud representation of the environment. (Rviz2 and Foxglove Studio)
  * Simple obstacle detection algorithm
<hr>

## Challenges
* Combining our obstacle avoidance program on the track with the pedestrian detection proved to be more complicated than initially expected
* Adapting the various nodes, creating unique publishers/subscribers, and implementing all of our code within ROS2
<hr>

## Final Project Videos
Click any of the clips below to reroute to the video. 

**Final Demo**

[<img src="images\finalDemo.PNG" width="600">](https://drive.google.com/file/d/1g-VeBIl5gdUQflPbHFEFg3UOX2X36orz/view?usp=drive_link)


**Final Clips**

Everything Together (3rd Person)

[<img src="images\everything.PNG" width="300">](https://drive.google.com/file/d/1p2i5tRBihoJaQEioC826KT2X-poMZBU0/view?usp=drive_link)

Everything Together (POV)

[<img src="images\pov.PNG" width="300">](https://drive.google.com/file/d/1Jt7cPzaUdMSlrA__SZ71dYU6QvQGNySb/view?usp=drive_link)

Obstacle Avoidance

[<img src="images\earlyAvoid.PNG" width="300">](https://drive.google.com/file/d/1Pd8OklPu9tDEfgzJNNkNkR_zGN-SYbMb/view?usp=drive_link)

Pedestrian Detection

[<img src="images\personDetected.PNG" width="300">](https://drive.google.com/file/d/11qNUHqOkSNa8mivNA_k-UFk8U7LfdoAp/view?usp=drive_link)

**Early Progress Clips**

Early Obstacle Avoidance

[<img src="images\earlierAvoid.PNG" width="300">](https://drive.google.com/file/d/14KP8B8-IEhhGi5ObEtc7LC3ndSPOTO66/view?usp=drive_link)

Early Pedestrian Detection

[<img src="images\earlyPed.PNG" width="300">](https://drive.google.com/file/d/1O3riZQaE1dmO9sFID_FuAqzC17s5VZ9x/view?usp=drive_link)

<hr>

## Hardware 

* __3D Printing:__ Camera Stand, Jetson Nano Case
* __Laser Cut:__ Base plate to mount electronics and other components.

__Parts List__

* Traxxas Chassis with steering servo and sensored brushless DC motor
* Jetson Nano
* WiFi adapter
* 64 GB Micro SD Card
* Adapter/reader for Micro SD Card
* Logitech F710 controller
* OAK-D Lite Camera
* LD19 Lidar (LD06 Lidar)
* VESC
* Point One GNSS with antenna
* Anti-spark switch with power switch
* DC-DC Converter
* 4-cell LiPo battery
* Battery voltage checker/alarm
* DC Barrel Connector
* XT60, XT30, MR60 connectors

*Additional Parts used for testing/debugging*

* Car stand
* USB-C to USB-A cable
* Micro USB to USB cable
* 5V, 4A power supply for Jetson Nano

### __Mechanical Design Highlight__

__Baseplate__

<img src="images\BasePlate_1.webp" height="350"> <img src="images\BasePlate_2.webp" height="350">

__Camera Stand__

Camera Stand components were designed in a way that it's an adjustable angle and height This design feature offers versatility and adaptability, ensuring optimal positioning of the camera to capture desired perspectives and accommodate various environments or setups.

<img src="images\Camera_Stand_1.webp" height="160"> <img src="images\Camera_Stand_2.webp" height="160">
<img src="images\Camera_Stand_3.webp" height="350">

__Circuit Diagram__

Our team made use of a select range of electronic components, primarily focusing on the OAK-D Lite camera, Jetson NANO, a GNSS board / GPS, and an additional Seeed Studio XIAO nRF52840 Sense (for IMU usage).
Our circuit assembly process was guided by a circuit diagram provided by our class TAs.

<img src="images\circuitDiagram.PNG" height="300">

<hr>

## Software

### Overall Architecture
Our project was completed entirely with ROS2 navigation in python. The 'rclpy' package is being used to control the robot and our primary control logic consists of the Calibration, Person Detection, Lane Detection, Lane Guidance, and nodes.

- The **Calibration Node** was adapted from Spring 2022 Team 1 and updated for our use case. We strictly needed the gold mask to follow the yellow lines and implemented our own lane following code.
  
- The **Person Detection Topic** was fully created for our team's project implementation. The topic is created in our oakd_node.py inside the ucsd_robocar_sensor2_pkg. The oakd_node publishes two topics the first being a color image feed from the camera and the second being an integer value of 1s or 0s for whether it sees a person or not, respectively, using the depth AI's neural network. In order to get the oakd_node running with the rest of the car during navigation a switch must be created in the car_config.yaml file to launch the oakd_launch.launch.py file created in the ucsd_robocar_sensor2_pkg and the previous oakd node must be switched off. The person detection topic is subscribed to in the Lane Guidance Node which ultimately controls the vehicle, while the camera feed is subscribed to in the lane detection node which finds the yellow lines to follow. 

- The **Lane Detection Node** is used to control the robot within the track. We adapt the PID function to calculate the error and set new values to determine the optimal motion of the car to continue following the yellow lines in the lane. This is done by taking the raw camera image, using callibrated color values to detect yellow, and ultimately using the processed image to publish the control values that are subscribed by the lane guidance node.

- Ultimately, the "magic" happens within the **Lane Guidance Node** which is responsible for directly controlling car's movement. We have adapted the Lidar subscription from Spring 2022 Team 1 to detect obstacles within a particular viewing range in front of our car. The lane guidance node subscribes to lane detection node and our Person Detection nodes to correctly traverse the path. If no obstacles are detected, the car will simply continue its line following program, sticking to the yellow lines in the middle of the lane. If an obstacle is detected by the lidar, the car will correspondingly make a turn based on the object's angle and distance. As it routes around the object, the car continues to check for obstacles to avoid any collision and come back to the path. Additionally, if the subscription to the person_detected node is triggered as active, then the car knows there is a pedestrian in view and will stop.

### Obstacle Avoidance
We used the LD06 Lidar to implement obstacle avoidance within ROS2. The program logic is quite simple in that we are constantly scanning the 60 degrees in front of the robot. If an object is detected within our distance threshold, the robot will accordingly make a turn to avoid it. Our logic for selecting which direction to turn in is quite simple in that if the object is on the left side, we first turn right, and otherwise, we turn left. Both turning directions include a corrective turn to bring the robot back to the centerline of the track and continue lane following.

### Pedestrian Detection
We used the DepthAI package to implement pedestrian detection within ROS2. We took advantage of the Tiny YOLO neural network setup found within the examples. We filter through the detections to check strictly for a "person" with adjustable confidence levels. We found that a 60% confidence level worked pretty well for our project's use cases. Surprisingly, we found better results with real humans walking in front of the robot (it would detect their feet and be able to classify them as "person" objects). We were also able to successfully scan various printout images of people with high accuracy and success. The programming logic for pedestrian detection is very simple in that if a "person" has been detected in the image passed through by the camera, the VESC throttles are set to 0, stopping the car, until the person has moved out of the field of view.
<hr>

## Gantt Chart

<img src="images\gantt_chart.webp" height="500">
<hr>

## Course Deliverables
Here are our autonomous laps as part of our class deliverables:

* line following: https://photos.app.goo.gl/pQ7n9FB2srGJNkFz9 
* lane following: https://photos.app.goo.gl/pQ7n9FB2srGJNkFz9 
* GPS: https://youtu.be/92Q-JpYGPZk?si=UYrh6Mo9-b4TGgYO

Team 7's the weekly project status updates and final presentation:
[Team 7 weekly status updates](https://docs.google.com/presentation/d/e/2PACX-1vSWm0AW0yZ6IKFCnXcJtbBB0NPDEejXtwTStLtW3yOxjlFvpV0wWUp3y91MQgVq3j63RR5WNTfaSFZW/pub?start=false&loop=false&delayms=3000)

[Team_7_Final_Presentation](https://docs.google.com/presentation/d/e/2PACX-1vRGnL11PP4RDo87JKWF-kLgD4dVyRBdL_eSWTUIe0eLQumJOI_wawX6sBa7MOMksFe8tPUjdFZBWRRE/pub?start=false&loop=false&delayms=3000)
<hr>

## Project Reproduction
If you are interested in reproducing our project, here are a few steps to get you started with our repo:

<ol>
  <li>Clone this repository</li>
  <li>Replace the <i>ucsd_robocar_sensor2_pkg</i> and <i>ucsd_robocar_lane_detection2_pkg</i> in the default <i>ucsd_robocar_hub2</i> directory</li>
  <li> Calibrate Your Robot
    <ol>
      <li>Toggle <i>camera_nav_calibration</i> to 1 and <i>camera_nav</i> to 0 within <i>node_config.yaml</i></li>
      <li>Run <i>source_ros2</i>, <i>build_ros2</i>, and then <i>ros2 launch ucsd_robocar_nav2_pkg all_nodes.launch.py</i> </li>
      <li>Adjust sliders within GUI to ensure gold mask is clear with <b>NO</b> noise </li>
      <li>Toggle <i>camera_nav_calibration</i> to 0 and <i>camera_nav</i> to 1 within <i>node_config.yaml</i></li>
      <li>Update your PID and throttle values in <i>ros_racer_calibration.yaml</i></li>
    </ol>
  </li>
  <li>Run on Track</li>
    <ol>
        <li>Run <i>source_ros2</i>, <i>build_ros2</i>, and then <i>ros2 launch ucsd_robocar_nav2_pkg all_nodes.launch.py</i> </li>
    </ol>
</ol>

Alternatively you can refer to the `lane_guidance_node.py` and `lane_detection_node.py` programs in `ucsd_robocar_lane_detection2_pkg/ucsd_robocar_lane_detection2_pkg` to adapt our code as needed for your project. We have extensive comments through the code explaining what is happening. Additionally, if you search for <i>(Edit as Wanted)</i> in our code, we have listed the primary areas where one would want to adjust parameters to adapt the lidar usage, pedestrian detection logic, and more. Some consistent, but simple and relevant issues we encountered were ensuring file pathways were correct and making sure that all dependencies are installed.

**Best of luck!**

<hr>

## Acknowledgements
Special thanks to Professor Jack Silberman and TA Arjun Naageshwaran for delivering the course!
Thanks to Raymond on Triton AI giving suggestions on our project!
Thanks to Nikita on Triton AI providing support on razorIMU_9dof repo for IMU usage!

**Programs Referenced:**
* [UCSD Robocar Framework](https://gitlab.com/ucsd_robocar2)
* [DepthAI_ROS_Driver](https://github.com/luxonis/depthai-ros)
* [razorIMU_9dof](https://github.com/NikitaB04/razorIMU_9dof)

<hr>

## Contacts

* Winston Chou - w3chou@ucsd.edu | winston.ckhs@gmail.com | [LinkedIn](https://www.linkedin.com/in/winston-wei-han-chou-a02214249/)
* Amir Riahi - 
* Rayyan Khalid - 
