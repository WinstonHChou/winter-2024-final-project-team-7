import RazorIMU
import time 
import time
import serial
import yaml
import threading

config_path = "/home/projects/ros2_ws/src/razorIMU_9dof/razor.yaml"
imu = RazorIMU([config_path])
imu.start()
time.sleep(2)
print(imu.orient[])