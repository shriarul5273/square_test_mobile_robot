# square_test_mobile_robot

## Author shriarul643@gmail.com


""
This a package to test the moition of a mobile robot.It publishes Velocity commands to the robot though ros msg "/cmd_vel" of type geometry_msgs/Twist.

```
$ cd ~/catkin_ws/src 

$ cd ~/catkin_ws

$ catkin_make
```

## launch in the terminal
```
$ roslaunch square_test_mobile_robot square_test_mobile_robot.launch

$ rostopic echo /cmd_vel
```

Visulize the results in rviz or rostopic

# or 

## rosrun in terminal

This package has 4 cpp file and can be runed according to the needs

