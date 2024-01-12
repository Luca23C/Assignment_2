# Assignment 2 - Robot Navigation

The following picture shows the environment of this assignment:

<img src="https://github.com/Luca23C/Assignment_2/assets/97911589/10d43c3a-4280-4938-a9a7-1e94e7b9b02a" width="778" height="435">

The aim of this work was to develop three different type of ROS node:
-(A) A node which implements an action client, allowing the user to set a specific target with x and y coordinates, or to cancel it. Another request was to use the feedback/status of the action server to know when the target has been reached. Finally that node also publishes the robot position and velocity as a custom message (with a structure like: x, y, vel_x, vel_z), by relying on the values published on the topic called /odom.
-(B) A service node, that when called, returns the coordinates of the last target sent by the user.
-(C) A service node that subscribes to the robot’s position and velocity (by using the custom message previously built) and implements a server to retrieve the distance of the robot from the target and the robot’s average speed.

These nodes have been developed in Python.

## Node developed

In this section, it will be explained all node and its functions developed for this project.

### Node (A)
For developing any kind of code, it is useful to start with Pseudocode or Flowchart. These tools enable to design information and action related to each process that need to be executed in order to achieve the goal.

For this type of work, it was choosen the flowchart rapresentation for showing each process:

<img src="https://github.com/Luca23C/Assignment_2/assets/97911589/c06bd4c1-64d2-42fc-aafb-2f0d8fe4a048">

Name of the file node: `bug_ac.py`
In this node, an action client was developed. Upon activation, it displays an intuitive sort of interface on terminal, where the user can be enter the x and y target coordinates that the robot needs to reach.

<img src="https://github.com/Luca23C/Assignment_2/assets/97911589/e7968d78-4e2a-4a67-94d9-ca505643cb3b">

When the target message is created and sent to the action server, it is executed a specific function called `delete_goal()`. This particular function allow user to delete the target in case of unreachable position or something else. 
In this function there is a while loop, which wait two kind of cases: robot reach the goal or user delete goal. In the first situation, it was retrieved the status of the goal ('reached' in this case), while in the second case it was used a build-in function called `keyboard.Listener()`, which listen if a specific key of the keyboard is pressed ('esc' key in this case). Therefore, if user press 'esc' key the target become successfully deleted and it was retrieved the status of the goal ('deleted' in this case). In both situation, the node come back to the initial user interface.

Subsequently, it was built a custom message structure and it was used a Subscriber to get the information needed from the topic `/odom` and these were published with a Publisher on the topic `/custom_message` by using the custom message previously made.

### Node (B)
In this second step, it was created a custom service. Initially, it was built a .srv file for defining the request and the response of the service, then two nodes have been developed: `last_target_srv.py` and `last_target_clt.py`.
In the first one, it was defined a Subscriber to get information about the last target coordinates on the topic `/reaching_goal/goal` and these were assigned to the request field. On the other hand, the client node (`last_target_clt.py`) allow to call the service when the user need to know the last coordinates sent.

### Node (C)
Here, it was considered the same approach used for the second step (Node B). First, it was made a .srv file and then two nodes have been developed: `info_navigation_srv.py` and `info_navigation_clt.py`.
Inside the `info_navigation_srv.py` two Subscriber were employed to provide information about the current position and velocity of the robot, as well as the choosen target. This information was respectively read from the topics: `custom_message` and `/reaching_goal/goal`.
Subsequently, these data was used to calculate the parameter to retrieve like: distance and average speed.
To calculate the distance, it was applied the Euclidean distance between two point, in other words current position (x1, y1) and target position (x2, y2).

<img src="https://github.com/Luca23C/Assignment_2/assets/97911589/c5021cef-426a-4ace-b235-d81869a75b14">

To compute the average speed, it was used the average window technique. In the Subscriber callback it was defined and filled an array called `velocity_array` about the linear velocity of the robot. Then, when the client call this type of service, within the callback service function it was computed the average speed by this algorithm:

```python
# Calculate averege speed of the robot
    window_size = rospy.get_param("/avg_window")    # Get this parameter from launch file
    moving_avg = []
    count = 0
    while count < len(velocity_array) - window_size + 1:
        window = velocity_array[count : count + window_size]
        window_avg = round(sum(window)/window_size, 3)
        moving_avg.append(window_avg)
        count += 1

    avg_speed = sum(moving_avg)/len(moving_avg)
```

## How to run

First of all it is necessary to clone this repository inside your workspace, into your local machine, by using the following command:

```bash
$ git clone https://github.com/Luca23C/Assignment_2
```

Now it is importat to run this command in your workspace folder to build this package within your workspace:
```bash
$ catkin_make
```

Finally it is possible to run this project by typing this line:
```bash
$ roslaunch assignment_2_2023 assignment1.launch
```

This last line execute the launch file of the project, where there are all nodes developed excluding the two client node: `last_target_clt.py` and `info_navigation_clt.py`. By using the services (B) and (C) you just need to open another terminal and execute our client node.

If you would like to know the last target sent, you can just type:
```bash
$ rosrun assignment_2_2023 last_target_clt.py
```

If you would like to know the distance of the robot from the target and the robot's average speed, you can just type:
```bash
$ rosrun assignment_2_2023 info_navigation_clt.py
```

Finally, if you wish to view the custom message for analyzing the current position of the robot as well as the linear and angular velocity of the robot, you can simply type:
```bash
$ rostopic echo /custom_message
```

## Possible improvement
A first improvement is to attempt to resolve the issue related to the `on_press()` function (located into `bug_ac.py` node), where the else case has been commented out. This is commented out because, if the user employs the keyboard with another terminal, this function outputs the message `"Invalid input. If you want to delete the target, please press 'esc'"` to the main terminal, even if the intention is not to communicate with the main terminal to delete the goal. Therefore, the goal could be to isolate the main terminal that is waiting for the 'esc' key press to delete the goal from the other terminals.
Another improvement could be to create a better design for the graphical interface.