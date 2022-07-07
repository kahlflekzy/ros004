- [BEGINNING](#beginning)
  - [Introduction](#introduction)
  - [ActionLib](#actionlib)
    - [Intro](#intro)
    - [Actions](#actions)
    - [Client](#client)
    - [ActionLib and BehaviourTree](#actionlib-and-behaviourtree)
    - [With Turtlebot3](#with-turtlebot3)
- [REFERENCES](#references)

## BEGINNING
[Details](http://wiki.ros.org/rviz/UserGuide)

### Introduction
We try to move a robot to different poses. We get those poses and store them in a file. We try to obtain the robots 
initial pose for use by `amcl` for localization we also store this in a file.

There is a `goal` topic which lets you send goals to the robot over ROS. I dug out this topic and the message type it 
expects.

There also is an `initialpose` topic, which lets you set an initial pose to seed the localization system.

`rospack find [package]` returns the path to a package. `rospack` itself returns information about a package.

We will try to copy behaviortree folder from the path returned by `rospack find`, then build it and see if it will 
work, instead of cloning. Well it didn't work, so I had to clone again, and build

```
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
```

Then created a package
```
catkin_create_pkg p1 rospy roscpp behaviortree_cpp_v3 move_base std_msgs geometry_msgs
```
And built successfully using the `--pkg [pkg]` flag

Now the aim is to run the navigation simulations.

First `cd` into `src` and clone the repo from

```
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

Then run `catkin_make`. Maybe we should do this before creating our package so that we add the dependency to our 
package. 

Next, we create a launch file to run the `FakeNode`, setting the command below in the launch file doesn't appear to 
work (for now). So we instead manually do it before running the launch file.
```
export TURTLEBOT3_MODEL=burger
```

You can unset a variable by running 
```
unset [YOUR_VARIABLE]
```

Runs successfully for the `FakeNode`. I then created a launch file for the navigation simulation.

I will create a node which listens to the topic `/move_base_simple/goal` and prints it.
The node also subscribes to `/initialpose` and logs the info to console.

Next is to get 4 goals which I will use to create my BehaviorTree. I added handlers to my subscriber node which logs 
some goals to a file. See `node1.py` for details. 

Then next step would involve using `actionlib` to send commands.

### ActionLib
[Details](http://wiki.ros.org/actionlib)

#### Intro
From the instructions, we first add a dependency on `actionlib` to our package.

This step should ideally be done while creating the package. But well, we continue from here, adding it to the 
`package.xml` and `CMakeLists.txt`

To `package.xml`
```
<depend>actionlib</depend>
<depend>actionlib_msgs</depend>
```
and to `CMakeLists.txt`, the below. 
Particularly, I added `genmsg`, `actionlib` and `actionlib_msgs` below the others.

```
find_package(catkin REQUIRED genmsg actionlib actionlib_msgs)
```

Then I ran `catkin_make` and everything built successfully.

#### Actions
Next is to define my actions. 

For this, I created an _actions_ folder in the package. Then created a `.action` file with the appropriate 
sections for goal, result and feedback. 

Remember to separate the sections by a `---`. See the created file for details. Then I added the instructions below to 
`CMakeLists.txt` for building the action. Options in _[...]_ are dynamic not constrained.

I discovered the folder should have been names `action` not `actions`. Then I could simply have done
```
add_action_files(
   FILES
   [Action1].action
)
```

But well, I did the below. My `action` depended on `geometry_msgs` so I had to include it in the `generate_messages` 
block below.

```
add_action_files(DIRECTORY [actions] FILES [MoveRobot].action)
generate_messages(DEPENDENCIES geometry_msgs actionlib_msgs)
```

Then ran `catkin_make`. This generates 7 MSGs that are used by `actionlib`.

One of which is `MoveRobotAction.msg`. You can run the code below to see it and make sure it is seen by ROS.

```
rosmsg show p1/MoveRobotAction.msg
```

#### Client
The client and the server are quite straightforward. Examples are in the code. I only ran into a problem when running 
`catkin_make`. I was having issues with `rosrun` it seemed to stem from the `source`ing.

I eventually used the version below, and everything worked.
```
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3/
```

The server has a `publish_feedback` method which can publish intermittent progress status.

I would look more into this next, plus doing real robot work.

#### ActionLib and BehaviourTree
I created a header file with a `MoveRobot` node. In it, I had to specify an Input goal for sending goals (via xml), 
the node I used was a `BT::StatefulActionNode` which is used for Asynchronous or long-running tasks. I had to define 
the `onStart`, `onHalted` and `onRunning` methods.

Now all these are just for behaviour tree. I then subsequently began the task of defining an `actionlib` client in this 
node. The client sends instructions to the server and sets its status to _RUNNING_, then changes to _SUCCESS_ when the 
server indicates that operations have been completed.

Coupled with my limited knowledge of C++, this took me quite some time to implement.

But I did get everything done. The next tasking part was defining a call back for the feedbacks. This took a lot of 
debugging skills. But I finally figured it out. 

Particularly the usage of `boost::bind` and the inclusion of `using namespace [p1]`. See the file `header_nodes.h` 
for details. Suffice it to say, everything run as is. And thus the template should be copied and only method 
implementations changed.

The server was in Python, so I didn't have problems here including creating and publishing a feedback. I ran into 
errors while running `catkin_make` but they were easy to figure out.

Of course, I had to add my nodes to `CMakeLists.txt` and then run `rosrun` on the server and client. I created a launch 
file but this was throwing incoherent errors. 

This was quite tasking. Refer to `cnode2.cpp`, `header_nodes.h` and `pnode2.py` for details.

#### With Turtlebot3
**Important** When running the turtlebot navigation simulation, deactivate conda and export the model.

Today, I modified my codes to run from a launch file passing arguments via the `args` attrib for to the client node 
( which has the `BehaviorTree`), and the server node, which interacts with `gazebo`, `rviz`, and the other stacks.  

I then modified server node to read the stored goals from earlier on, and publish them on the `/move_base_simple/goal` 
topic.

I also then had to get `PoseWithCovarianceStamped` messages from the `/initialpose` topic, store them in a particular 
format on file using the previous `node1.py` 

Then in the server, I read them back, parsed them and published to the topic. And it works. See the note below and the 
files for details.

**Important**: I had to add a `rospy.sleep(2)` before my `pose` message was published on the `initialpose` topic.

#### Topics to watch out for
- `/move_base/current_goal`
- `/move_base/result`
- `/move_base/status`

So I eventually settled for using `/move_base/status`. I subscribed to the topic then added attributes and callbacks 
which perform some actions, like note the goal ID and status and update a variable which is used by the method 
executing a goal. I had a bug which I eventually figured out. Particularly, I had been making a comparison between goal 
ID and goal status, which never matched up. It made some task never execute particularly tasks after returning to base.
Third tasks, usually and following. I added a hook for setting the initial pose for the robot for `amcl`.

The initial implementation made it compulsory. But the update I made, made it now optional.
This was important, because once it has been initialized, it was no good to set it again while `rviz` and `gazebo` 
weren't shutdown. So particularly, I added the line `<arg name="[arg_name]"/>` to the launch file. Then To the server 
node, I added the attribute `args="$(arg arg_name)"` and when running the launch file, the command now looks as below. 
With _0_ for don't initialize and _1_ for initialize the pose.

```
roslaunch p1 l2.launch init_base:=1
```

I also edited the tree xml definition to include more goals and didn't include the name attribute for some tree nodes, 
cos I checked, and it was optional. Everything runs satisfactorily so far.  See the files `pnode2.py`, `tree10.xml` and 
`l2.launch` for details.

## REFERENCES
1. [Callbacks](https://get-help.robotigniteacademy.com/t/ros-action-client-with-feedback-using-classes/7625/3)
2. [ActionLib](http://wiki.ros.org/actionlib/Tutorials)
3. [ActionLib Tutorials](http://wiki.ros.org/actionlib_tutorials)
4. [argc & argv](https://stackoverflow.com/a/2601548)
5. [Launch File Args](http://wiki.ros.org/roslaunch/XML/node#Attributes)
6. [ROS Launch Tools](http://wiki.ros.org/roslaunch/Commandline%20Tools)