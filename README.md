The software is for low-level control of a mobile robot.

Short description of each file:

1. lowCtrlDimRob.cpp
This is the application code for the low-level control of hydraulic system for the robot.
It is supposed to run in Real-Time OS (i.e. Xenomai patched Linux).
Two real-time tasks with different priorities are initiated.

a. task_rt (high priority):
-Read sensor inputs.
-Set to corresponding state machine status.
-Send actuator outputs
-Desired motor command is passed to non-real time loop.

b. task_nrt (low priority):
-Send the desired motor command to the motor controller.
-Acquire the sensors inputs and the actuators outputs from RT thread and publish the data via Robot Operating System (ROS) protocol.


2. highCtrlDimRob.cpp
This is the application code for the high-level control of the robot.
Gamepad(joy_node) message is subscribed, and when Gamepad is opereated, some of desired actuator signals are published to the low-level controller. 

3. stateMachine.cpp
Finite State Machine (FSM) implementations.

4. pciAnalogy.cpp
The code has functions to perform application features on DAQ card (e.g. acquire or generate signals)

5. pciBase.cpp
The code has functions to perform basic features on DAQ card (e.g. configure or reset DAQ card)
