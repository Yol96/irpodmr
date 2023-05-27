#!/usr/bin/python

import rospy
import math
import random

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn


class Follower:
    def __init__(self, turtle_leader_name, turtle_follower_name, follower_speed):
        self.turtle_leader_name = turtle_leader_name
        self.turtle_follower_name = turtle_follower_name
        self.follower_speed = follower_speed

        # Init turtles callbacks
        self.leader_turtle_pose = Pose()
        self.follower_turtle_pose = Pose()
        self.subscribe(self.turtle_leader_name, self.leaderPoseCallback)
        self.subscribe(self.turtle_follower_name, self.followerPoseCallback)

        self.pub = rospy.Publisher("/" + self.turtle_follower_name + "/cmd_vel", Twist, queue_size=10)

        # Spawn the second turtle
        spawner = rospy.ServiceProxy('spawn', Spawn)
        spawner(random.uniform(0, 11), random.uniform(0, 11), 0, turtle_follower_name)

        # Update seconds turtle position
        self.updateFollowerPostion()


    def subscribe(self, turtle_name, callback):
        rospy.Subscriber("/" + turtle_name + "/pose", Pose, callback)


    def leaderPoseCallback(self, data):
        self.leader_turtle_pose = data


    def followerPoseCallback(self,data):
        self.follower_turtle_pose = data


    def updateFollowerPostion(self):
        leader_position = self.getLeaderPosition()

        self.follower_turtle_pose.x = leader_position.x
        self.follower_turtle_pose.y = leader_position.y

    
    def getLeaderPosition(self):
        return rospy.wait_for_message("/" + self.turtle_follower_name + "/pose", Pose)


    def follow(self):
        while not rospy.is_shutdown():
            x_diff = self.leader_turtle_pose.x - self.follower_turtle_pose.x
            y_diff = self.leader_turtle_pose.y - self.follower_turtle_pose.y

            distance = math.hypot(x_diff, y_diff)
            
            angle_to_target = math.atan2(y_diff, x_diff)
            angle_diff = angle_to_target - self.follower_turtle_pose.theta

            cmd_vel = Twist()
            cmd_vel.linear.x = self.follower_speed * distance
            cmd_vel.angular.z = 8.0 * angle_diff

            self.pub.publish(cmd_vel)
            rospy.sleep(0.1)


if __name__ == '__main__':
    rospy.init_node('tom_and_jerry_node')

    follower_speed = rospy.get_param('~follower_speed')
    turtle_leader_name = rospy.get_param('~first_turtle_name', 'turtle1')
    turtle_follower_name = rospy.get_param('~second_turtle_name', 'turtle2')

    follower = Follower(turtle_leader_name, turtle_follower_name, follower_speed)
    follower.follow()
