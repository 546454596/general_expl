#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Header

if __name__ == '__main__':
    rospy.init_node('init_server')
    
    # NodeHandle
    node = rospy
    # nh = rospy.get_param('~')
    # Publishers
    pos_cmd_pub = rospy.Publisher("/position_cmd", PositionCommand, queue_size=50)
    # Parameters
    pub_traj_id_ = rospy.get_param("~init_server/pub_traj_id", 0)
    init_pos_x = rospy.get_param("~init_server/init_x", 0)
    init_pos_y = rospy.get_param("~init_server/init_y", 0)
    init_yaw = rospy.get_param("~init_server/init_yaw", 0)

    # Control parameter
    cmd = PositionCommand()
    cmd.kx = [3.7, 3.7, 4.2]
    cmd.kv = [1.4, 1.4, 2.0]
    rospy.loginfo("Start time: %s", rospy.Time.now().to_sec())
    
    cmd.header = Header()
    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = "world"
    cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
    cmd.trajectory_id = pub_traj_id_
    cmd.position.x = init_pos_x
    cmd.position.y = init_pos_y
    cmd.position.z = 0.0
    cmd.velocity.x = 0.0
    cmd.velocity.y = 0.0
    cmd.velocity.z = 0.0
    cmd.acceleration.x = 0.0
    cmd.acceleration.y = 0.0
    cmd.acceleration.z = 0.0
    cmd.yaw = init_yaw
    cmd.yaw_dot = 0.0
    
    rospy.sleep(2.0)
    for i in range(100):
        cmd.position.z += 0.01
        pos_cmd_pub.publish(cmd)
        rospy.sleep(0.03)
    
    """for i in range(50):
        cmd.position.z -= 0.01
        pos_cmd_pub.publish(cmd)
        rospy.sleep(0.02) """
    rospy.logwarn("[Init server]: beginning initialization!")
    rospy.loginfo("End time: %s", rospy.Time.now().to_sec())
