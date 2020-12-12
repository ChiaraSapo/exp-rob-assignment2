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

turns = 0
LOOPS = 1
counter = 0

global subscriberNORM
global subscriberPLAY


image_pub = rospy.Publisher(
    "/output/image_raw/compressed", CompressedImage, queue_size=1)

vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

camera_pub = rospy.Publisher(
    "/head_position_controller/command", Float64, queue_size=1)

pub = rospy.Publisher(
    "tempPlay", Float64, queue_size=1)


def callbackNORM(ros_data):
    global subscriberNORM
    time.sleep(1)

    vel_Norm = Twist()
    vel_Norm.linear.x = 0
    vel_Norm.linear.y = 0
    vel_Norm.linear.z = 0
    vel_Norm.angular.x = 0
    vel_Norm.angular.y = 0
    vel_Norm.angular.z = 0

    rospy.loginfo('entered img NORM fnct')

    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    greenLower = (50, 50, 20)
    greenUpper = (70, 255, 255)

    blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        rospy.loginfo('img NORM fnct: ball')
        rospy.set_param('ball', 1)
        time.sleep(1)
        subscriberNORM.unregister()

    else:
        for i in range(0, 10):
            vel_Norm.angular.z = 0.2
            vel_pub.publish(vel_Norm)
            time.sleep(1)
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None

            if len(cnts) > 0:
                rospy.loginfo('img NORM fnct: ball after turning')
                rospy.set_param('ball', 1)
                time.sleep(1)
                subscriberNORM.unregister()

        rospy.loginfo('img NORM fnct: no ball')
        rospy.set_param('ball', 0)
        time.sleep(1)
        subscriberNORM.unregister()


def callbackPLAY(ros_data):
    global subscriberPLAY
    rospy.loginfo('entered PLAY img fnct')

    vel_Play = Twist()
    vel_Play.linear.x = 0
    vel_Play.linear.y = 0
    vel_Play.linear.z = 0
    vel_Play.angular.x = 0
    vel_Play.angular.y = 0
    vel_Play.angular.z = 0

    time.sleep(3)

    #### direct conversion to CV2 ####
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

    greenLower = (50, 50, 20)
    greenUpper = (70, 255, 255)

    blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    # cv2.imshow('mask', mask)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        rospy.loginfo('img PLAY fnct: ball')

        # set counter back to zero
        counter = 0

        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 9:
            rospy.loginfo('img PLAY fnct: radius > 9')
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(image_np, (int(x), int(y)), int(radius),
                       (0, 255, 255), 2)
            cv2.circle(image_np, center, 5, (0, 0, 255), -1)

            vel_Play.angular.z = -0.002*(center[0]-400)
            vel_Play.linear.x = -0.01*(radius-100)
            vel_pub.publish(vel_Play)

        elif radius < 7:
            rospy.loginfo('img PLAY fnct: radius < 7')
            vel_Play.linear.x = 0.5
            vel_pub.publish(vel_Play)

        elif radius > 7 and radius < 9:
            rospy.loginfo('img PLAY fnct: 7 < radius < 9')
            vel_Play.angular.z = 0
            vel_Play.linear.x = 0
            vel_pub.publish(vel_Play)

            # Rotate camera
            vel_camera = Float64()
            vel_camera.data = 0
            while vel_camera.data < 6:  # to check
                vel_camera.data = vel_camera.data + 0.1
                camera_pub.publish(vel_camera)
            rospy.set_param('camera_turns', 1)

    else:
        rospy.loginfo('img PLAY fnct: cannot see ball, i will turn')
        counter = counter+1
        vel_Play.angular.z = 0.5
        vel_pub.publish(vel_Play)

    if counter > 5:
        rospy.loginfo('img PLAY fnct: exceeded counter')
        vel_Play.linear.x = 0
        vel_Play.angular.z = 0
        vel_pub.publish(vel_Play)
        rospy.set_param('counter', 1)
        subscriberPLAY.unregister()

    cv2.imshow('window', image_np)
    cv2.waitKey(2)  # Python libs


def dog(target):

    time.sleep(3)

    client = actionlib.SimpleActionClient(
        '/robot_reaching_goal', exp_assignment2.msg.PlanningAction)

    client.wait_for_server()

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


class MIRO_Sleep(smach.State):

    # Init function for smach machine sleep state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['normal_command'])

    def execute(self, userdata):

        # go home and sleep
        time.sleep(3)
        rospy.loginfo('sleep: go home')
        # dog([0, 0, 0])
        # time.sleep(5)

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
        global subscriberNORM

        for i in range(0, 10):

            # Go rand (then set rand)
            time.sleep(3)
            rospy.loginfo('normal: going rand')
            #dog([-5, 5, 0])

            # Look for ball
            time.sleep(3)
            rospy.loginfo('normal: looking for ball')
            subscriberNORM = rospy.Subscriber("camera1/image_raw/compressed",
                                              CompressedImage, callbackNORM,  queue_size=1)

            while rospy.get_param('ball') == 2:
                time.sleep(0.5)

            ball = rospy.get_param('ball')
            rospy.set_param('ball', 2)

            # Case 0: no ball --> continue in normal state
            if ball == 0:
                time.sleep(3)
                rospy.loginfo(
                    'normal: i see no ball, i will continue doing my things')

            # Case 1: ball --> go in play state
            elif ball == 1:
                time.sleep(3)
                rospy.loginfo(
                    'normal: i do see ball, i will chase it: set play')

                c = 'play_command'
                return c

            # Case err: ball wasn't set yet
            else:
                time.sleep(3)
                rospy.loginfo('normal: ball was not set yet')

        return 'sleep_command'


# Play state of the smach machine.


class MIRO_Play(smach.State):

    # Init function for smach machine play state.
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['normal_command'])

    def execute(self, userdata):
        global subscriberPLAY
        time.sleep(3)
        rospy.loginfo('play: chase ball')

        pub.publish(1)

        #subscriberPLAY = rospy.Subscriber("camera1/image_raw/compressed", CompressedImage, callbackPLAY,  queue_size=1)
        while rospy.get_param('counter') == 0:
            time.sleep(1)
        if rospy.get_param('counter') == 5:
            return 'normal_command'

        rospy.set_param('counter', 0)

        time.sleep(4)

        c = 'normal_command'
        return c

# Ros node that implements a state machine with three states: sleep, play, normal.
# It also initializes the home postion and the arrival parameters.


def main():

    rospy.init_node('state_manager')

    rospy.set_param('ball', 2)
    rospy.set_param('counter', 0)

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
