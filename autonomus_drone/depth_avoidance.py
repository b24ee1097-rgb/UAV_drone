import rclpy  #python library to run ros that helps me create nodes publishers subscribers and nodes
from rcply.nodes import Node   #calling nodes
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy , DurabilityPolicy #quality of system ->contuosil;y sends commands until they are recieved by the reciever
from px4_msgs.msg import(  #px4 based drone used here
    OffboardControlMode,
#position
# velocity
# acceleration
# attitude
# body_rate
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


