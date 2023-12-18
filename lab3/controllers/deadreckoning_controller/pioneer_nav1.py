# PioneerNavigation Class Definition
# File: pioneer_nav1.py
# Date: 18th Oct 2022
# Description: Simple Navigation Class support (2022)
# Author: Terry Payne (trp@liv.ac.uk)

from controller import Supervisor, Node

import math
import pose
from enum import Enum

class MoveState(Enum):
    STOP = 0
    FORWARD = 1
    ARC = 2
    CONFIGURE = 3

class PioneerNavigation:
    """ A custom class to initialise and manage simple navigation on a Pioneer Adept robot """

    #WHEEL_RADIUS = 0.0975 # in meters
    #AXEL_LENGTH = 0.31    # in meters
    WHEEL_RADIUS = 0.0957 # in meters - found using CONFIGURE
    AXEL_LENGTH = 0.323   # in meters- found using CONFIGURE

    def __init__(self, robot):
        
        self.robot = robot                        # reference to the robot
        self.robot_node = self.robot.getSelf()    # reference to the robot node
        self.robot_pose = self.get_real_pose()   # the robots believed pose, based on real location
        self.state = MoveState.STOP
        self.configure_str="Configuring..."
    
        # enable motors
        self.left_motor = self.robot.getDevice('left wheel')
        self.right_motor = self.robot.getDevice('right wheel')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        
        # Initialise motor velocity
        self.left_motor.setVelocity(0.0);
        self.right_motor.setVelocity(0.0);   

    def get_real_pose(self):
        if self.robot_node is None:
            return pose.Pose(0, 0, 0)
            
        real_pos = self.robot_node.getPosition()
        rot = self.robot_node.getOrientation()
        theta = math.atan2(-rot[0], rot[3])
        halfpi = math.pi / 2
        theta2 = theta + halfpi
        if (theta > halfpi):
            theta2 = -(3 * halfpi) + theta
        
        return pose.Pose(real_pos[0], real_pos[1], theta2)
    

    def configure_initialise_parameters(self, wheel_omega):
        # This rotates the robot about one wheel, and monitors the time and distance
        # taken to rotate around a circle
        pose = self.get_real_pose()
        self.velocity = wheel_omega
        self.config_max_x = pose.x
        self.config_min_x = pose.x
        self.config_timer = 0
        self.config_prev_theta = pose.theta
        
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(self.velocity)
        self.state = MoveState.CONFIGURE

 
    def configure_check_parameters(self, timestep):
        pose = self.get_real_pose()
        axel_length = self.config_max_x - self.config_min_x # represents a circle travelled by center of robot
        
        if (self.config_prev_theta < 0) and (pose.theta >= 0.0):
            # We have just passed a heading of 0
            distance = axel_length * 2 * math.pi # i.e. circumference around icr
            wheel_radius = distance / (self.velocity * (self.config_timer / 1000))
            self.configure_str = f"axel_length={axel_length:.3f} wheel_radius={wheel_radius:.3f}"
            self.configure_initialise_parameters(self.velocity)
            self.config_timer = 0
        else:
            self.config_max_x = max(self.config_max_x, pose.x)
            self.config_min_x = min(self.config_min_x, pose.x)
        
        self.config_timer += timestep

        self.config_prev_theta = pose.theta
        return self.configure_str

    def forward(self, target_dist, robot_linearvelocity):
        wheel_av = (robot_linearvelocity/self.WHEEL_RADIUS)
        target_time = target_dist/robot_linearvelocity
        
        self.left_motor.setVelocity(wheel_av)
        self.right_motor.setVelocity(wheel_av)
        self.state = MoveState.FORWARD
        
        # return target_time as millisecs          
        return 1000.0*target_time

    def arc(self, icr_angle, icr_r, icr_omega):
        target_time = icr_angle / icr_omega

        # Calculate each wheel velocity around ICR
        vl = icr_omega * (icr_r - (self.AXEL_LENGTH / 2))
        vr = icr_omega * (icr_r + (self.AXEL_LENGTH / 2))
        
        leftwheel_av = (vl/self.WHEEL_RADIUS)
        rightwheel_av = (vr/self.WHEEL_RADIUS)
        
        self.left_motor.setVelocity(leftwheel_av)
        self.right_motor.setVelocity(rightwheel_av)
        self.state = MoveState.ARC

        # return target_time as millisecs          
        return 1000.0*target_time            
    def stop(self):
    
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.state = MoveState.STOP
    