                                                                                                 
#  1st ASSIGNMENT of Research Track 


This project implements a ros2 application that allows to control two turtle nodes in a GUI (turtlesim_node). The user can choose which turtle to move and
apply to it an angular and/or linear velocity. Furthermore the software comprehends a mechansim to prevent the turtle from colliding with each other or the border.

# Nodes
- turtlesim_node ->     creates the GUI where turtles can move, provides the basic topics to publish velocities and retrieve turtles position + orientation and spawns the first turtle
- spawn_node ->         places the second turtle in the GUI with associated topics
- ui_node ->            manages user input, turtles movement and prevents eventual collisions with each other or the border
- distance_node ->      retrieves the position and orientation of turtles in real time, other than their distance, and publishes them on the topic shared with ui_nod

# Topics
Default topics:
- turtle/pose
- turtle/cmd-vel
Custom topics:
- distance: used to publish distance between turtles
- boundary_condition_on_1: used to publish the orientation of turtle1 that leads to border collision when near it
- boundary_condition_on_2: used to publish the orientation of turtle2 that leads to border collision when near it
- relative_pos: used to publish relative position and orientatio among the two turtles to avoid collision

# Requirements
- ros2 framework

# Instructions to run
After downloading the files and directories from GitHub into a folder, move it to src/ repository in yout ros2 workspace.
Then in a ros2 terminal:
1. colcon build ->                      executed in workspace directory to compile the package
2. source local_setup.bash ->           run inside install/ folder
3. ros2 run <\package_name> <\node> ->  to execute actual nodes (one terminal for every node)

# Instructions to use
Follow the textual interface:
1. select a turtle (1 or 2)
2. insert an angular velocity
3. insert a linear velocity
and again...

# Functioning
The application, through the ui_node, starts asking the user which turtle wants to move, the linear and angular velocity for the specified turtle, which will be applied for a second. The rotation is executed right away, however before applying the linear velocity the program checks for possible future collision. The first check is about mutual collision, it's evaluated if the distance among the two turtles is lower than a constant predefined threshold. After the first evaluation, the application controls for the possibility to hit the border. If both conditions are cleared then the drone moves, otherwise its motion is stopped or not executed at all. Indeed, a turtle may start moving, but then it can get stopped if a check isn't passed, since these evaluations are made in real-time every 10 milliseconds.

To comprehend in more depth how the application works I suggest viewing the code comments.

# Functions
Distance.cpp:
- topic_callback1: retrieves position of turtle1 from topic "turtle1/pose"
- topic_callback2: retrieves position of turtle1 from topic "turtle2/pose"
- timer_callback1: computes and publishes distance among two turtles on topic "distance"
- timer_callback2: computes the boundary conditions for turtle1 checking its orientation and position and then it publishes it on topic "boundary_condition_on_1"
- timer_callback3: computes the boundary conditions for turtle2 checking its orientation and position and then it publishes it on topic "boundary_condition_on_2"
- timer_callback4: computes relative position among two turtles and then publishes it on topic "relative_pos"
UI.cpp:
- main_loop: implements the main function from which all the other functions gets called directly or indirectly
- user_input: displays textual interface for choosing which turtle to move and inserting both angular and linear velocity
- rotate_turtle: applies angular velocity for one second (stopped by timer_callback1)
- timer_callback1: stops the rotation after one second (called by rotate_turtle)
- move_turtle_x: checks if no collision can verify itself(constraints_check) and then moves the turtle for one second (stopped by timer_callback2)
- timer_callback2: stops the translation (calls stop_linear_motion) after one second (called by move_turtle_x)
- stop_linear_motion: actual function that stops the translation, a specific function is needed, since it can be called in different situation
- topic_callback1: retrieves distance from topic "distance"
- topic_callback2: retrieves condition on boundary for turtle1 from topic "boundary_condition_on_1"
- topic_callback3: retrieves condition on boundary for turtle2 from topic "boundary_condition_on_2"
- topic_callback4: retrieves relative position amogn turtle1 and turtle2 from topic "relative_pos"
- stop_movement_in_progress: stops translation after movement is started (calls stop_linear_motion)
- constraints_check: checks collision among two turtles (calls motion_would_collide) and then with boundaries (calls boundary_violation on both turtles)
- motion_would_collide: checks relative position of two turtles to see if there could be a collision (called by constraints_check)
- boundary_violation: checks if the borders could be hit by a turtle (called by constraints_check)

# Author
Daneri Gregorio

# -----------> Have fun!!! <-----------
