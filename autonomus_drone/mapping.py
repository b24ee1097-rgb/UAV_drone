import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import math
from sensor_msgs.msg import Image, LaserScan
from px4_msgs.msg import VehicleLocalPosition
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, TransformStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge

def quatornion_to_euler(roll, pitch , yaw):
  cr = math.cos(roll/2)
  sr = math.sin(roll/2)
  cy = math.cos(yaw/2)
  sy = math.sin(yaw/2)
  cp = math.cos(pitch/2)
  sp = math.sin(pitch/2)
  qw = cr * cp * cy + sr * sp * sy
  qx = sr * cp * cy - cr * sp * sy
  qy = cr * sp * cy + sr * cp * sy
  qz = cr * cp * sy - sr * sp * cy
  return qx, qy, qz, qw
  #now we will assign color using the lidar height scan that we are getting from the drone and we will assign color to the point cloud based on the height of the sca  to visualise it in 3d
  def height_to_color(h, max_h):
   val = min(max(h/max_h,0.0),1.0) #scalling from 0 to 1
   c = ColorRGBA()
   c.r = 0.0
   c.g = 2.0*val
   c.b = 1.0 - 2.0*val

#    BRESENHEMS ALGORITHM
def besenhem(x1,y1,x2,y2):
    points = [];
    m = (y1-y2)/(x1-x2)
    if abs(m) < 1:
        m = - m

   for( x = x1;x<x2;x++):
        y = round(m*(x)+c)
        points.append((x,y))
        