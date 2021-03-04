#! /usr/bin/env python

# @file human_commands.py
# @brief This node implements an actionlib client to move the ball-


import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkState
from tf import transformations
import math
import random
import actionlib
import actionlib.msg
import exp_assignment2.msg
import rospy
import time

# This function implements a client for the ball motion server.


def human_client():

    client = actionlib.SimpleActionClient(
        '/reaching_goal', exp_assignment2.msg.PlanningAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = exp_assignment2.msg.PlanningGoal()

    goal.target_pose.pose.position.x = random.randrange(0, 9)  # -5
    goal.target_pose.pose.position.y = random.randrange(0, 9)  # 2
    goal.target_pose.pose.position.z = random.choice([0, 1, 10])

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    return client.get_result()

# Ros node that calls the client for the ball motion server


def main():

    rospy.init_node('human_client')

    while True:
        human_client()
        time.sleep(10)


if __name__ == '__main__':
    main()
