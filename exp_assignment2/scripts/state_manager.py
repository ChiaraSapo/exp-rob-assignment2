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

        # Give command home
        sleep_command = 'go_home'

        # Publish sleep command
        pub.publish(sleep_command)
        time.sleep(4)

        # Change state
        c = 'normal_command'
        return c

# Normal state of the smach machine.


class MIRO_Normal(smach.State):

    # Init function for smach machine normal state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['sleep_command', 'play_command'])

    # Smach machine state normal actions:
    # Listens to user: if user says "Play" or "Hey buddy" it outputs command to enter play state.
    # If user says nothing, it goes to random positions for a while (n loops) then outputs command to enter sleep state.
    # @return c: command to switch between states.
    def execute(self, userdata):

        # Set state parameter
        rospy.set_param('state', 'NORMAL')

        for i in range(0, LOOPS):

            # Checks if user is speaking

            # If "ball" is calling MIRO, enter play state

            # Else wander around
            else:
                normal_command = 'go_rand'

                # Publish normal command
                pub.publish(normal_command)
                time.sleep(3)

            # Randomly decide to sleep, enter sleep state
            if random.randrange(0, 5) == 1:
                c = 'sleep_command'
                return c

        return 'sleep_command'

# Play state of the smach machine.


class MIRO_Play(smach.State):

    # Init function for smach machine play state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['normal_command'])

    # Smach machine state play actions:
    # Looks at user, saves his coordinates as next position, publishes them (goes toward the human).
    # It then listens to the user. If user says "go to posx posy", publishes the coordinates (goes to the point).
    # If user says "Hey buddy" or "Play" it waits. If user says nothing, it looks for the user gesture to go somewhere,
    # and publishes the coordinate he receives (goes to the point).
    # This repeates for a while (n loops) then the robot enters normal state again.
    # @return c: command to switch between states.
    def execute(self, userdata):

        # Set state parameter
        rospy.set_param('state', 'PLAY STATE')

        for i in range(0, LOOPS):

            # Look for ball
            # Go to ball

        c = 'normal_command'
        return c

# Ros node that implements a state machine with three states: sleep, play, normal.
# It also initializes the home postion and the arrival parameters.


def main():

    rospy.init_node('state_manager')

    # Set parameters
    # Home position
    rospy.set_param('home_posx', 3)
    rospy.set_param('home_posy', 3)

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
