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
from cv_bridge import CvBridge #from opencv

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
    def update(self , d_f,d_r,d_l,l_f,l_r,l_l):
        return(
        self.fused_front = self.fuse_measurements(d_f, l_f)
        self.fused_right = self.fuse_measurements(d_r, l_r)
        self.fused_left = self.fuse_measurements(d_l, l_l)
        )
class SmartObstacleNavigator(Node):
    def __init__(self):
        super().__init__('smart_obstacle_navigator')
        
        self.declare_parameters("", [("flight_altitude", 5.0)])
        self.flight_altitude = float(self.get_parameter("flight_altitude").value)
        
        # Drone-specific physical constraints (x500 model)
        self.MIN_SAFE_CLEARANCE = 0.7  # meters - based on 0.5m rotor span + safety
        self.CORRIDOR_THRESHOLD = 3.0   # consider corridor if both sides < 3m

        self.waypoints = []
        self.wp_index = 0
        self.manual_goal = False
        self.goal_pose = None
        self.nav_mode = NavigationMode.PATH_FOLLOWING
        self.last_path_hash = None
        
        self.phase = FlightPhase.INIT
        self.armed = False
        self.have_local_position = False

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.offboard_counter = 0

        self.raw_depth = {"f": None, "l": None, "r": None}
        self.raw_lidar = {"f": None, "l": None, "r": None}
        self.last_depth_time = 0.0
        self.last_lidar_time = 0.0

        self.fused_front = 20.0
        self.fused_left = 20.0
        self.fused_right = 20.0
        self.fusion = SensorFusionModule(self.get_logger())

        self.filtered_center_err = 0.0
        self.prev_yaw = 0.0
        
        self.current_forward_vel = 0.0
        self.current_side_vel = 0.0
        
        self.stuck_counter = 0
        self.position_history = deque(maxlen=50)
        self.last_waypoint_time = 0.0

        # Enhanced features
        self.consecutive_failures = 0
        self.last_progress_check = 0.0
        self.last_progress_distance = 0.0
        self.yaw_history = deque(maxlen=20)
        self.lateral_history = deque(maxlen=20)

                # Wide space handling
        self.last_significant_clearance = 0.0
            qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
            qos_sensor = QoSProfile(  # QoS is defined in ROS2 but qos_sensor i have defined it here
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,#keep last reading
                depth=5, #keep last 5 measurements from the sensors to help with the fusion and obstacle avoidance
            )
            
            qos_viz = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history = HistoryPolicy.KEEP_LAST,
                depth=1,
            )

            self.offboard_pub = self.create_publisher(
                OffboardControlMode, "/fmu/in/offboard_control_mode", qos_cmd
            )
            self.setpooint_pub = self.create_publisher(
                TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_cmd
            )
            self.command_pub = self.create_publisher(
                TrajectorySetpoint, "/fmu/in/vehicle_command", qos_cmd
            )
            self.viz_path_pub = self.create_publisher(Path,"/planned_path", qos_viz) #it is the data meant for rviz (QoS(qualtiy of sefvice) :->viz)
            
        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.pos_cb, qos_cmd
        )
        #subscriptions we use to recieve the data from publishers
        self.create_subscription(LaserScan, "/scan", self.lidar_cb, qos_sensor)
        self.create_subscription(Path, "/global_path", self.path_cb, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)
        self.bridge = CvBridge() #Important Cvbridge 
        self.timer = self.create_timer(0.1, self.control_loop)
        #logger we use for real time debugging and to see the values of the sensors and the state of the drone
