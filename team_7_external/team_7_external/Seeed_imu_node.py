import RazorIMU
import time 
import time
import serial
import yaml
import threading

config_path = "/home/projects/ros2_ws/src/razorIMU_9dof/razor.yaml"

if __name__ == "__main__":
    razor_imu = RazorIMU('razor.yaml')
    razor_imu.start()
    time.sleep(2)
    for t in range(1,20):
        print("X: " + str(razor_imu.orient[AxisRef.X.value]))
        print("Y: " + str(razor_imu.orient[AxisRef.Y.value]))
        print("Z: " + str(razor_imu.orient[AxisRef.Z.value]))
        time.sleep(1)
    razor_imu.shutdown()
    print("All Good!")