# Introduction
The package exp_assignment2 simulates a robot dog moving on a gridded plane. He can either sleep in his kennel, wander around or play with a ball controlled by a simulated user. The project is implemented with a ROS structure, written in Python, and simulated through Gazebo.
Movements are implemented with an actionlib client-server structure, and vision is implemented via cv.

# Simulated environment appearance
On Gazebo we can see an arena with a man sitting on a chair, a big green ball and a dog.
<p align="center">
  <img height="500" width="500" src="https://github.com/ChiaraSapo/exp-rob-assignment2/blob/master/exp_assignment2/images/Screenshot%20from%202020-12-12%2017-25-55.png?raw=true "Title"">
</p>

## Structure of the dog
The dog has two wheels, a castor wheel, a fixed neck, a head able to rotate around the jaw axis, and a camera on top of the head. The wheels are controlled with a differential drive approach and the head can be rotated through a position control. 
<p align="center">
  <img height="500" width="500" src="https://github.com/ChiaraSapo/exp-rob-assignment2/blob/master/exp_assignment2/images/Screenshot%20from%202020-12-12%2017-26-24.png?raw=true "Title"">
</p>


# software architecture and state diagrams
The architecture is based on four ROS nodes that implement:
- a smach state machine to simulate sleep, normal and play states
- a server to move the ball
- a server to move the robot
- a client to move the ball

## Ros messages and parameters
In order to communicate, the two actionlib client-server structures use an action message (Planning.action) that comprises:
- goal: geometry_msgs/PoseStamped target_pose
- result: uint32 reached_position
- feedback: string stat, geometry_msgs/Pose position

Moreover, the structure uses some ROS parameters, such as: 
- ball: if set to 0: no ball was seen. if set to 1: ball was seen. init/std value: 2.
- counter: if set to 10: ball was not seen for 10 iterations. init/std value: 0. 
- rotate_camera: if set to 1: robot has reached the ball and has to turn its head. init/std value: 0.


# Package
There are multiple folders:
- action: contains the action file
- config: contains the config file (for the motor of the robot)
- images: contains the images that can be seen on this README
- launch: contains the launch file 
- scripts: contains the python files, which are described in detail in the next paragraph
- urdf: contains the xacro and urdf files (of robot, ball, human)
- words: contains the world for the gazebo simulation

## Python files 

### go_to_point_ball.py
Ros node that implements an actionlib server to move the ball. The ball can move in the arena or disappear from the robot's sight.

### go_to_point_robot.py
Ros node that implements an actionlib server to move the robot. The robot can move in the arena.

### human_commands.py
Ros node that implements an actionlib client to move the ball. 

### state_manager.py
Ros node that implements a smach state machine with three different states. 
Sleep: the dog goes to its kennel in position [0,0,0] and stays there for a while. Then there is a state transition to'normal'. 
Normal: the dog goes to a random position, then looks around. If it sees the ball, it enters in play state. If it doesn't see it, it goes to another random position. After a while, if nothing has happened, it goes in sleep state.
Play: the dog follows the ball around. When it reaches it, it turns the head to the left and to the right. When it can't see the ball for a few seconds, the the state transitions to normal.

#### Internal classes and functions:
- class find_follow_ball.
This class initializes the publishers and then subscribes to the camera image. It looks for the ball through the camera: if it is visible, the robot approaches it and waits a few moments to turn the robot's head. If the ball is not visible the robot rotates on the spot to look for it. After a few moments without seeing the ball, a ros parameter is set so that the state transitions to normal and the callback function is set to sleep. 
<p align="center">
  <img height="500" width="500" src="https://github.com/ChiaraSapo/exp-rob-assignment2/blob/master/exp_assignment2/images/Screenshot%20from%202020-12-12%2017-26-38.png?raw=true "Title"">
</p>

- function find_ball.
This is a callback function to the normal state's subscriber to the camera image. It looks for the ball though the camera. If it sees it, it sets a ros parameter so that the state transitions to play. If it doesn't see it, the same parameter is set to a different value so that the state remains in normal.
- function move_dog
This function implements a client to move the dog to desired location. It has as input the desired location and as output the result of the action.

# Instalation and running procedure:
Download the package in your_catkin_ws/src folder. To install it:
```sh
cd "Your catkin workspace"/src/exp_assignment2
chmod +x install.sh
./install.sh
```
To run it:
```sh
roslaunch exp_assignment2 gazebo_world.launch
```

# System limitations
The robot moves very slowly.
Sometimes ball moves in front of the dog exactly when image function is looking for it and the robot doesn't see it.
When camera rotates, the opencv window freezes until rotation is complete. It also happens in switching from play to normal.
For testing purposes, the ball moves along line x=-5 and robot moves only in (5,-5,0) when it is in normal state. 
