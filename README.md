## Introduction
The package exp_assignment2 simulates a robot dog moving on a gridded plane. He can either sleep in his kennel, wander around or play with a ball controlled by a simulated user. The project is implemented with a ROS structure, written in Python, and simulated through Gazebo.
Movements are implemented with an actionlib client-server structure, and vision is implemented via cv.

## Simulated environment appearance
On Gazebo we can see an arena with a man sitting on a chair, a big green ball and a dog.

## Structure of the dog
The dog has two wheels, a castor wheel, a fixed neck, a head able to rotate around the jaw axis, and a camera on top of the head. The wheels are controlled with a differential drive approach and the head can be rotated through a position control. 




go_to_point_ball.py
Ros node that implements an actionlib server to move the ball.

go_to_point_robot.py
Ros node that implements an actionlib server to move the robot.

human_commands.py
Ros node that implements an actionlib clientto move the ball.

state_manager.py
Ros node that implements a smach state machine with three different states. 
Sleep: the dog goes to its kennel in position [0,0,0] and stays there for a while. Then there is a state transition to'normal'. 
Normal: the dog goes to a random position, then looks around. If it sees the ball, it enters in play state. If it doesn't see it, it goes to another random position. After a while, if nothing has happened, it goes in sleep state.
Play: the dog follows the ball around until it can't be seen anymore, the the state transitions to normal.

