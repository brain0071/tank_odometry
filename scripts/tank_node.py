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


class TANK_Odometry():
    def __init__(self):
        
        rospack = rospkg.RosPack()
        
        data_path = rospack.get_path("tank_odometry") + '/scripts/calibration_test.csv'  # in real tank
        self.tags = genfromtxt(data_path, delimiter=',')  # home PC
        self.tags = self.tags[:, 0:4]
        
        self.tank_pos_pub = rospy.Publisher('tank_pos_vel', Odometry, queue_size=1)
        
        # update attitude and rotation 
        rospy.Subscriber("/auto_grasp/filter/quaternion",QuaternionStamped,self.updateAtt_callback,
                         queue_size=1)           
        
        rospy.Subscriber("/apriltag_pose_array", AprilTagDetectionArray, self.estimate_callback, 
                         queue_size=1)
        
        self.rotation = np.eye(3)
        self.att = np.array([1., 0., 0., 0.])  # Quaternion format: qw, qx, qy, qz

        self.last_pos_tank = np.array([0, 0, 0])
        self.last_time = time.time()

        rospy.spin()

    # Quaternion to rotation
    def q_to_rot_mat(self):

        qw, qx, qy, qz = self.att[0], self.att[1], self.att[2], self.att[3]
        rot_mat = np.array([
            [1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
            [2 * (qx * qy + qw * qz), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qw * qx)],
            [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx ** 2 + qy ** 2)]])
        return rot_mat

    # update current attitude and rotation matrix
    def updateAtt_callback(self, msg):
        q_enu = [msg.quaternion.w,  msg.quaternion.x,  msg.quaternion.y,  msg.quaternion.z]
        self.att = q_enu
        self.rotation = self.q_to_rot_mat(self.att)

    # estimate position in tank Coordinate system, and velocity in body Coordinate system 
    # NOTE: tank Coordinate system coincides with xsens Coordinate system
    def estimate_callback(self, msg):

        global x, y, z
        x_total = []
        y_total = []
        z_total = []

        for i, tag in enumerate(msg.detections):
        
            tag_id = int(tag.id[0])

            index = np.where(self.tags[:, 0] == tag_id)

            # From t265 camera Coordinate system  transform to tank Coordinate system (Xsens frame)
            # x:forward y:left  z:up
            y_body = tag.pos.x
            x_body = tag.pos.y
            z_body = tag.pos.z

            # NOTE: t265 camera Coordinate system 
            # https://dev.intelrealsense.com/docs/intel-realsensetm-visual-slam-and-the-t265-tracking-camera
            # Xsens Coordinate system  
            # https://mtidocs.movella.com/getting-started-2
            
            pos_tmp = np.dot(self.rotation, [x_body, y_body, z_body])  
            
            x_total.append(self.tags[index, 1] + pos_tmp[0]) 
            y_total.append(self.tags[index, 2] + pos_tmp[1]) 
            z_total.append(self.tags[index, 3] + pos_tmp[2])  

            # rospy.loginfo("Detected Apriltags: %d, %f, %f, %f", 
                        # tag_id, self.tags[index, 1], self.tags[index, 2], self.tags[index, 3])
        
        # 
        x = sum(x_total) / len(x_total)
        y = sum(y_total) / len(y_total)
        z = sum(z_total) / len(z_total)
        
        # get velocity in body Coordinate system (update 5hz)
        t = time.time() - self.last_time
        vel_x_tank = (x - self.last_pos_tank[0]) / t
        vel_y_tank = (y - self.last_pos_tank[1]) / t
        vel_z_tank = (z - self.last_pos_tank[2]) / t
        
        velocity = np.dot(np.linalg.pinv(self.rotation), 
                          np.array([vel_x_tank[0], vel_y_tank[0], vel_z_tank[0]]))
        
        self.last_pos_tank[0] = x
        self.last_pos_tank[1] = y
        self.last_pos_tank[2] = z
        self.last_time = time.time()

        # pos in tank
        pos_msg = Odometry()
        pos_msg.pose.pose.position.x = x
        pos_msg.pose.pose.position.y = y
        pos_msg.pose.pose.position.z = z

        #NOTE: There is a steady-state error.
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
