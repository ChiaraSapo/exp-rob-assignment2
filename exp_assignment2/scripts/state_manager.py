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
import sys
from scipy.ndimage import filters
import imutils
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64, UInt32

VERBOSE = False
LOOPS = 2
vel_camera = Float64()
MAX_COUNTER = 25
SEARCH_FOR_BALL = 35
global counter
global subscriberNORM
global subscriberPLAY

# Publisher

vel_pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)


# This function is the callback for the normal state.
# It checks if the ball is visible from the camera. If it is: it sets the ros parameter ball=1.
# If it  isn't: it sets the ros parameter ball=0.


def find_ball(ros_data):

    global subscriberNORM, vel_Norm, SEARCH_FOR_BALL

    vel_Norm = Twist()

    # Init velocity
    vel_Norm.linear.x = 0
    vel_Norm.linear.y = 0
    vel_Norm.linear.z = 0
    vel_Norm.angular.x = 0
    vel_Norm.angular.y = 0
    vel_Norm.angular.z = 0

    # rospy.loginfo('entered img NORM fnct')

    # Convert to cv2
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    # Color limits
    greenLower = (50, 50, 20)
    greenUpper = (70, 255, 255)

    # Create masks
    blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find contour
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    # Only proceed if at least one contour was found
    if len(cnts) > 0:
        # Ball was found
        rospy.loginfo('ball was seen!')
        rospy.set_param('ball', 1)
        subscriberNORM.unregister()

    else:
        vel_Norm.angular.z = 0.3
        vel_pub.publish(vel_Norm)

        # Ball was not found
        rospy.set_param('ball', 0)
        subscriberNORM.unregister()


# This class is used to detect and follow the green ball in the arena.
class find_and_follow_ball:

    # It initializes a publisher to the image, one to the robot velocity and one to the camera motor.
    # It subscribes to the camera image
    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''

        # Init publishers
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("/robot/cmd_vel",
                                       Twist, queue_size=1)
        self.camera_pub = rospy.Publisher(
            "/robot/joint_position_controller/command", Float64, queue_size=1)

        # Subscribed Topic
        self.subscriber = rospy.Subscriber("/robot/camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)

    # This function is the callback function of the subscription to the camera image.
    # It looks for a green contour in the image, plots a circle around it and makes the robot approach it.
    # When the robot has the object at a specified distance, it stops, turns its head twice and again looks for the object.
    # If the robot doesn't see the ball for 10 iterations in a row, it sets the ros parameter counter to 10 and then waits
    # for it to be zero again.
    def callback(self, ros_data):
        global counter
        global vel_camera
        global MAX_COUNTER

        vel_Play = Twist()

        # Init velocity
        vel_Play.linear.x = 0
        vel_Play.linear.y = 0
        vel_Play.linear.z = 0
        vel_Play.angular.x = 0
        vel_Play.angular.y = 0
        vel_Play.angular.z = 0

        # Read counter ros parameter: proceed only if it's not max
        counter = rospy.get_param('counter')
        while counter == MAX_COUNTER:
            time.sleep(1)

        if VERBOSE:
            print('received image of type: "%s"' % ros_data.format)

        # Convert to cv2
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Color limits
        greenLower = (50, 50, 20)
        greenUpper = (70, 255, 255)

        # Create masks
        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Find contour
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        # Only proceed if at least one contour was found
        if len(cnts) > 0:

            # Find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # Only proceed if the radius meets a minimum size
            if radius > 10:

                # Draw the circle and centroid on the frame, then update the list of tracked points
                cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)

                # Publish robot vel
                vel_Play.angular.z = -0.002*(center[0]-400)
                vel_Play.linear.x = -0.01*(radius-100)
                self.vel_pub.publish(vel_Play)

                # When robot has arrived: turn head
                if vel_Play.linear.x <= 0.05 and vel_Play.angular.z <= 0.05:

                    # Stop robot completely
                    vel_Play.angular.z = 0
                    vel_Play.linear.x = 0
                    self.vel_pub.publish(vel_Play)

                    # Rotate camera
                    rospy.set_param('rotate_camera', 1)

            # Go near ball
            else:
                vel_Play = Twist()
                vel_Play.linear.x = 0.5
                self.vel_pub.publish(vel_Play)

        # Look for ball by turning on the spot
        else:
            vel_Play = Twist()
            vel_Play.angular.z = 0.5
            self.vel_pub.publish(vel_Play)

            # Increase counter of iterations without seeing the ball
            counter = counter+1
            rospy.set_param('counter', counter)
            time.sleep(1)

            # If counter is max: stop
            if counter == MAX_COUNTER:
                vel_Play.angular.z = 0
                self.vel_pub.publish(vel_Play)
                self.subscriber.unregister()

        # Show camera image
        cv2.imshow('window', image_np)
        cv2.waitKey(2)

# This function is a client for the robot motion server. It sends the desired position.


def move_dog(target):

    time.sleep(3)

    client = actionlib.SimpleActionClient(
        '/robot_reaching_goal', exp_assignment2.msg.PlanningAction)

    client.wait_for_server()

    rospy.loginfo('Going to %d %d %d', target[0], target[1], target[2])

    # Creates a goal to send to the action server.
    goal = exp_assignment2.msg.PlanningGoal()
    goal.target_pose.pose.position.x = target[0]
    goal.target_pose.pose.position.y = target[1]
    goal.target_pose.pose.position.z = target[2]

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    rospy.loginfo('arrived, exiting dog fnct')

    return client.get_result()

# Sleep state of the smach machine.


class MIRO_Sleep(smach.State):

    # Init function for smach machine sleep state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['normal_command'])

    # Smach machine state sleep actions:
    # Calls dog() function to move towards the kennel, waits some seconds and exits state.
    # @return c: command to switch between states.
    def execute(self, userdata):

        # Go home and sleep
        time.sleep(3)
        rospy.loginfo('sleep: go home')
        move_dog([0, 0, 0])
        time.sleep(5)

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
    # In a loop:
    # Calls move_dog() function to go to a random position, then subscribes to the camera image. It reads
    # the ball ros parameter: if it's set to 2 it waits. If it's set to 0, it moves the robot to another
    # random position. If it's set to 1 it switches to play state.
    # At the end of the loop: it switches to sleep state.
    # @return c: command to switch between states.
    def execute(self, userdata):

        global subscriberNORM, LOOPS

        # Loop to wander around
        for i in range(0, LOOPS):

            # Go rand (then set rand)
            time.sleep(3)
            rospy.loginfo('normal: going rand')
            move_dog([random.randrange(0, 9), random.randrange(0, 9), 0])

            # Loop to look for ball
            for j in range(0, SEARCH_FOR_BALL):

                # Look for ball
                time.sleep(3)
                rospy.loginfo('normal: looking for ball')
                subscriberNORM = rospy.Subscriber("camera1/image_raw/compressed",
                                                  CompressedImage, find_ball,  queue_size=1)

                # Param not set yet
                while rospy.get_param('ball') == 2:
                    time.sleep(1)

                # Save param value
                ball = rospy.get_param('ball')
                rospy.set_param('ball', 2)

                # Case 0: no ball --> continue in normal state
                if ball == 0:
                    rospy.loginfo('normal: no ball: keep rotating')
                    time.sleep(2)

                # Case 1: ball --> go in play state
                elif ball == 1:
                    rospy.loginfo(
                        'normal: i do see ball: set play')
                    c = 'play_command'
                    return c

            rospy.loginfo(
                'normal: i see no ball, i will continue minding my own business')

        c = 'sleep_command'
        return c


# Play state of the smach machine.


class MIRO_Play(smach.State):

    # Init function for smach machine play state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['normal_command'])

        self.camera_pub = rospy.Publisher("/robot/joint_position_controller/command",
                                          Float64, queue_size=1)

    # Smach machine state play actions: find and follow the ball.
    # If robot reaches the ball and stops: rotate camera.
    # If robot can not see ball for a while (counter=MAX_COUNTER): switch to normal state.
    # @return c: command to switch between states.
    def execute(self, userdata):

        global subscriberPLAY
        global MAX_COUNTER

        time.sleep(3)
        rospy.loginfo('play: chase ball')

        # Find and follow green ball
        ic = find_and_follow_ball()

        while rospy.get_param('counter') < MAX_COUNTER:
            # If robot has stopped in front of ball
            if rospy.get_param('rotate_camera') == 1:

                rospy.loginfo('rotating camera')
                vel_camera.data = 0

                # CORRECTED ERROR

                for turn in range(0, 5):
                    vel_camera.data = vel_camera.data + 0.1
                    self.camera_pub.publish(vel_camera)
                    time.sleep(1)

                for turn2 in range(0, 10):
                    vel_camera.data = vel_camera.data - 0.1
                    self.camera_pub.publish(vel_camera)
                    time.sleep(1)

                for turn3 in range(0, 5):
                    vel_camera.data = vel_camera.data + 0.1
                    self.camera_pub.publish(vel_camera)
                    time.sleep(1)

                vel_camera.data = 0
                self.camera_pub.publish(vel_camera)
                time.sleep(5)

                # Rotation finished
                rospy.set_param('rotate_camera', 0)

                # Wait until dog can't see the ball no more, and switch to normal state
                rospy.set_param('counter', 0)

        rospy.loginfo('back to normal, havent seen ball for a while')
        c = 'normal_command'
        return c


# Ros node that implements a state machine with three states: sleep, play, normal.
# It also initializes the ball and counter parameters.


def main():

    rospy.init_node('state_manager')

    # Init ball ros param to 2 and counter ros param to 0.
    rospy.set_param('ball', 2)
    rospy.set_param('counter', 0)
    rospy.set_param('rotate_camera', 0)

    time.sleep(3)

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
    cv2.destroyAllWindows()
    sis.stop()


if __name__ == '__main__':
    main()
