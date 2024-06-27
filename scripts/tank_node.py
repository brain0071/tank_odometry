#!/usr/bin/env python

import numpy as np
import rospkg
import rospy
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import QuaternionStamped

from tank_odometry.msg import AprilTagDetectionArray
from tank_odometry.msg import AprilTagDetection

from numpy import genfromtxt
import ekf_class


# use EKF
class TANK_Odometry():
    """
    estimation position and velocity only use AprilTags
    Xsens only for Coordinate system transformation 
    """
    def __init__(self):
        
        rospack = rospkg.RosPack()
        data_path = rospack.get_path("tank_odometry") + '/scripts/calibration_tank.csv'  # in real tank
        self.tags = genfromtxt(data_path, delimiter=',')  # home PC
        self.tags = self.tags[:, 0:4]
        self.state_dim = 3
        self.cov_mat = 0.05
        
        self.ekf = ekf_class.ExtendedKalmanFilter()
        
        self.tank_pos_pub = rospy.Publisher('tank_pos_vel', Odometry, queue_size=1)    
        rospy.Subscriber("/apriltag_pose_array", AprilTagDetectionArray, self.estimate_callback,
                         queue_size=1)
        
        self.last_pos_tank = np.array([0, 0, 0])
        self.last_time = time.time()
        rospy.spin()

    # estimate position and velocity in tank Coordinate system
    # NOTE: tank Coordinate system coincides with xsens Coordinate system
    def estimate_callback(self, msg):
        
        current_time = time.time()
        self.ekf.prediction()
        # get length of message
        num_meas = len(msg.detections)
        # if new measurement: update particles
        if num_meas >= 1:
            # (num_meas, 4)
            measurements = np.zeros((num_meas, 1 + self.state_dim))


        for i, tag in enumerate(msg.detections):
            tag_id = int(tag.id[0])
            tag_distance_cam = np.array(([tag.pose.pose.pose.position.x * 1.05,
                                          tag.pose.pose.pose.position.y * 1.1,
                                          tag.pose.pose.pose.position.z]))
            measurements[i, 0] = np.linalg.norm(tag_distance_cam)
            
            
            index = np.where(self.tags[:, 0] == tag_id)

            measurements[i, 1:4] = self.tags[index, 1:4]
        
        self.ekf.update(measurements)
        estimated_position = self.ekf.get_x_est()
        
        x = estimated_position[0]
        y = estimated_position[1]
        z = estimated_position[2]    
    
        # get velocity in body Coordinate system (update 5hz)
        t = current_time - self.last_time
        
        # rospy.loginfo("Last position: %f, %f, %f and time: %f", self.last_pos_tank[0], self.last_pos_tank[1], self.last_pos_tank[2], t)
        
        vel_x_tank = (x - self.last_pos_tank[0]) / t
        vel_y_tank = (y - self.last_pos_tank[1]) / t
        vel_z_tank = (z - self.last_pos_tank[2]) / t
        
        velocity = [vel_x_tank, vel_y_tank, vel_z_tank]
        # velocity = np.dot(np.linalg.pinv(self.rotation), 
        #                   np.array([vel_x_tank[0], vel_y_tank[0], vel_z_tank[0]]))

        self.last_pos_tank = [x, y, z]
        self.last_time = current_time

        # pos in tank
        pos_msg = Odometry()
        pos_msg.pose.pose.position.x = x
        pos_msg.pose.pose.position.y = y
        pos_msg.pose.pose.position.z = z

        pos_msg.twist.twist.linear.x = velocity[0]
        pos_msg.twist.twist.linear.y = velocity[1]
        pos_msg.twist.twist.linear.z = velocity[2]

        rospy.loginfo("Current position: %f, %f, %f", 
                      pos_msg.pose.pose.position.x, pos_msg.pose.pose.position.y, pos_msg.pose.pose.position.z)
        
        rospy.loginfo("Current velocity: %f, %f, %f", 
                      pos_msg.twist.twist.linear.x, pos_msg.twist.twist.linear.y, pos_msg.twist.twist.linear.z)
        
        self.tank_pos_pub.publish(pos_msg)

def main():

    rospy.init_node('tank_node')
    TANK_Odometry()

if __name__ == '__main__':
    main()
