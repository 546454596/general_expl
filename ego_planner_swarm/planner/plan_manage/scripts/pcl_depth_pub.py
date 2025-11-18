#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf.transformations as tf

class DepthPub:
  def __init__(self):
    rospy.init_node('pcl_depth_pub', anonymous=True)
    self.depth_sub = rospy.Subscriber('/ddk/rgbd/depth/image_raw', Image, self.callback)
    self.pcl_depth_pub = rospy.Publisher('/pcl_render_node/depth', Image, queue_size=1000)
    self.odom_sub = rospy.Subscriber('/ddk/ground_truth/odom', Odometry, self.odomcallback)
    self.sensor_odom_pub = rospy.Publisher('pcl_render_node/sensor_pose', PoseStamped, queue_size = 1000)
    self.cam2body = np.array([[0.0, 0.0, 1.0, 0.0], 
                    [-1.0, 0.0, 0.0, 0.0], 
                    [0.0, -1.0, 0.0, 0.0], 
                    [0.0, 0.0, 0.0, 1.0]])
  
  def callback(self, data):
    depth_msg = data
    depth_msg.header.frame_id = "SQ01s/camera"
    self.pcl_depth_pub.publish(depth_msg)

  def pose_to_matrix(self, pose):
    translation = [pose.position.x, pose.position.y, pose.position.z]  
    rotation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    rotation_matrix = tf.quaternion_matrix(rotation)
    rotation_matrix[0:3,3] = translation
    return rotation_matrix
  
  def matrix_to_pose(self, matrix):
    translation = matrix[0:3, 3]
    rotation = tf.quaternion_from_matrix(matrix)
    pose = PoseStamped()
    pose.pose.position.x = translation[0]
    pose.pose.position.y = translation[1]
    pose.pose.position.z = translation[2]
    pose.pose.orientation.x = rotation[0]
    pose.pose.orientation.y = rotation[1]
    pose.pose.orientation.z = rotation[2]
    pose.pose.orientation.w = rotation[3]
    return pose
  
  def odomcallback(self, odom_data):
    # 将Pose转换为4x4矩阵
    pose_matrix = self.pose_to_matrix(odom_data.pose.pose)
    #将cam2body矩阵与pose矩阵相乘
    transformed_matrix = np.dot(pose_matrix, self.cam2body)
    # 将变换后的矩阵转换回PoseStamped
    pose_stamped_msg = self.matrix_to_pose(transformed_matrix)
    pose_stamped_msg.header = odom_data.header
    # 发布消息
    self.sensor_odom_pub.publish(pose_stamped_msg)


if __name__ == '__main__':

  try:
    pcl_depth =  DepthPub() 
    rospy.spin()
  except rospy.ROSInterruptException:
    pass      
