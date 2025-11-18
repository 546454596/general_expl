#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
import math

class GoalPublisher:
    def __init__(self):
        rospy.init_node('goal_publisher', anonymous=True)

        self.goals = [
            Point(5.0, 5.0, 1.0),
            Point(0.0, -3.0, 1.0),
            Point(-4.0, 0.0, 1.0),
            Point(0.0, 3.0, 1.0),
            Point(4.0, 0.0, 1.0),
            Point(3.0, -3.0, 1.0)
        ]

        self.goal_index = 0
        self.goal_tolerance = 1.2  # Distance threshold to consider goal reached

        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10, latch=True)
        self.odom_subscriber = rospy.Subscriber('/drone_0_visual_slam/odom', Odometry, self.odom_callback)
      
        rospy.loginfo("Goal publisher initialized.")
        self.publish_goal(self.goals[0])

    def odom_callback(self, msg):
        if self.goal_index >= len(self.goals):
            rospy.loginfo("All goals have been reached.")
            rospy.signal_shutdown("Completed navigation.")
            return

        current_position = msg.pose.pose.position
        goal_position = self.goals[self.goal_index]

        distance = self.calculate_distance(current_position, goal_position)

        if distance < self.goal_tolerance:
            rospy.loginfo(f"Goal {self.goal_index + 1} reached: {goal_position}")
            self.goal_index += 1

            if self.goal_index < len(self.goals):
                rospy.sleep(0.5)
                self.publish_goal(self.goals[self.goal_index])
        else:
            rospy.loginfo_throttle(5, f"Navigating to Goal {self.goal_index + 1}. Distance: {distance:.2f}")

    def calculate_distance(self, current, goal):
        return math.sqrt((goal.x - current.x) ** 2 + (goal.y - current.y) ** 2)

    def publish_goal(self, point):
        goal = PoseStamped()
        goal.header.frame_id = "world"
        goal.header.stamp = rospy.Time.now()

        goal.pose.position.x = point.x
        goal.pose.position.y = point.y
        goal.pose.position.z = point.z

        goal.pose.orientation.w = 1.0  # Neutral orientation

        self.goal_publisher.publish(goal)
        rospy.loginfo(f"Published new goal: {point}")

if __name__ == '__main__':
    try:
        node = GoalPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")
