#!/usr/bin/env python


# @file state_manager.py
# @brief This node implements a smach state machine to simulate a dog that can sleep, play and wander around.

import roslib
import rospy
import smach
import smach_ros
import time
import random
from std_msgs.msg import String
import matplotlib.pyplot as plt
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkState
from tf import transformations
import math
import actionlib
import actionlib.msg
import exp_assignment2.msg


def dog(target):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient(
        '/robot_reaching_goal', exp_assignment2.msg.PlanningAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = exp_assignment2.msg.PlanningGoal()
    goal.target_pose.pose.position.x = target[0]  # random.randrange(0, 9)
    goal.target_pose.pose.position.y = target[1]  # random.randrange(0, 9)
    goal.target_pose.pose.position.z = target[2]  # -1

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    return client.get_result()

# Sleep state of the smach machine.


class MIRO_Sleep(smach.State):

    # Init function for smach machine sleep state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['normal_command'])

    # Smach machine state sleep actions:
    # Publishes "go home" command, waits ("sleeps") and outputs command to enter normal state
    # @return c: command to switch between states.
    def execute(self, userdata):

        dog([1, 1, 0])

        # Change state
        c = 'normal_command'
        return c

# Normal state of the smach machine.


class MIRO_Normal(smach.State):

    # Init function for smach machine normal state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['sleep_command', 'play_command'])

    def execute(self, userdata):

        dog = ([random.randrange(0, 11), random.randrange(0, 11), 0])
        c = 'sleep_command'
        return c


# Play state of the smach machine.


class MIRO_Play(smach.State):

    # Init function for smach machine play state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['normal_command'])

    def execute(self, userdata):

        # Set state parameter
        rospy.set_param('state', 'PLAY STATE')

        c = 'normal_command'
        return c

# Ros node that implements a state machine with three states: sleep, play, normal.
# It also initializes the home postion and the arrival parameters.


def main():

    rospy.init_node('state_manager')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])

    with sm:
        # Add states to the container
        smach.StateMachine.add('SLEEP', MIRO_Sleep(),
                               transitions={'normal_command': 'NORMAL'})
        smach.StateMachine.add('NORMAL', MIRO_Normal(),
                               transitions={'sleep_command': 'SLEEP',
                                            'play_command': 'PLAY'})
        smach.StateMachine.add('PLAY', MIRO_Play(),
                               transitions={'normal_command': 'NORMAL'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
