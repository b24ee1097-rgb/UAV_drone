import rclpy  #python library to run ros that helps me create nodes publishers subscribers and nodes
from rclpy.nodes import Node   #calling nodes
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy , DurabilityPolicy #quality of system ->contuosil;y sends commands until they are recieved by the reciever
from px4_msgs.msg import(  #px4 based drone used here
    OffboardControlMode,
#position
# velocity
# acceleration
# attitude
# body_rate
#depth_avoidance is for obstacle detection and avoidance using depth sensors and lidar
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from cv_bridge import CvBridge

import numpy as np
import math
from enum import IntEnum  #kind of gibving refderences to some quantities these are named integers 
from collections import deque

class FlightPhase(IntNum):
   INIT = 0
   TAKEOFF = 1
   NAVIGATE = 2
   REACHED = 3

class NavigationMode(IntNum):
   PATH_FOLLOWING = 0
   EXPLORATION = 1

def point_to_segment_distance(px, py, x1, y1, x2, y2):
    A = px - x1
    B = py - y1
    C = x2 - x1
    D = y2 - y1
    dot = A * C + B * D
    len_sq = C * C + D * D 
    if len_sq == 0:
        return math.hypot(px - x1, py - y1)
    u = max(0.0, min(1.0, dot / len_sq))
    x = x1 + u * C
    y = y1 + u * D
    return math.hypot(px - x, py - y)


class SensorFusionModule:
    def __init__(self,logger): #logger tells us about the sensor information
        self.logger = logger  #so these are the reliabilities of the sensors 
        self.depth_reliability = 0.92
        self.lidar_reliability = 0.65
        self.measurement_noise_depth = 0.2
        self.measurement_noise_lidar = 0.3
    def fuse_measurements(self, d_depth, d_lidar):v\are the depth that i got from the cameras and the depth that i am getting from the lidar
        if depth_measurement is None and lidar_measurement is None:
            self.logger.warning("No valid measurements from either sensor.")
            #RGB-D camera (like Intel RealSense) must have been used by the drone to get the depth measurement
            d_ok = (depth_measurement is not None) and (depth_measurement > 0.1 and depth_measurement < 10.0) #removing the faulty errors
            l_ok = (lidar_measurement is not None) and (lidar_measurement > 0.1 and lidar_measurement < 10.0)
            if d_ok and l_ok :
                w_d = self.depth_reliability/self.measurement_noise_depth #these are the weights for the depth and lidar measurements based on their reliabilities and measurement noise
                w_l = self.lidar_reliability/self.measurement_noise_lidar
                #now i will got to averaging 
                fused = (w_d*d_depth + w_l*d_lidar)/(w_d+ w_l)
                if abs(d_depth-d_lidar) > 2.0:
                    fused = min(d_depth, d_lidar)
                return fused
            if d_ok:
              return d_depth
            if l_ok:
              return d_lidar
        return 20.0
