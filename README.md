# Software Description

## General Architecture

The software architecture is built upon ROS (Robot Operating System) and structured around three primary nodes:

* *LineFollowerNode (Arduino-based)*: Controls robot navigation using a PID algorithm guided by infrared sensors.
* *DispenserNode (Arduino-based)*: Operates a medication dispenser by activating a stepper motor in response to commands.
* *ReaderNode (Python on Raspberry Pi)*: Central node managing communication, event logic, and timing between other nodes.

The ROS Master (roscore) coordinates node communication and message-passing.

---

## Node Descriptions

### LineFollowerNode

* *Purpose:* Provide precise line-following navigation using PID control.
* *Inputs:* Digital signals from TCRT5000 infrared sensors.
* *Outputs:* PWM motor signals (via L298N driver), serial "stop" signal to ReaderNode when reaching a stopping point.

### DispenserNode

* *Purpose:* Accurately dispense medication using stepper motor activation.
* *Inputs:* Serial "dispense" command from ReaderNode; optional patient-confirmation button input.
* *Outputs:* Stepper motor rotation, serial acknowledgment "dispensed" sent back to ReaderNode.

### ReaderNode

* *Purpose:* Coordinates communication, timing, and logic between LineFollowerNode and DispenserNode.
* *Inputs:* Serial "stop" and "dispensed" signals; GPIO-based button input.
* *Outputs:* Serial "go" (to LineFollowerNode) and "dispense" (to DispenserNode) commands.

---

## Message Interfaces

Communication between nodes occurs via serial:

* *LineFollowerNode → ReaderNode:* "stop" when robot reaches dispensing point.
* *ReaderNode → LineFollowerNode:* "go" to resume robot navigation.
* *ReaderNode → DispenserNode:* "dispense" initiates medication dispensing.
* *DispenserNode → ReaderNode:* "dispensed" indicates successful dispensing or patient confirmation.

---

## Launch System

A basic ROS launch file is used to initialize ROS Master and start the ReaderNode:

xml
<launch>
  <node pkg="roslaunch" type="roscore" name="roscore" output="screen" />
  <node pkg="robot_control" type="reader_node.py" name="reader_node" output="screen" />
</launch>


---

## Behavior and Logic

The ReaderNode uses an event-driven state machine to manage the following logic:

* Upon receiving a "stop" from LineFollowerNode:

  1. Starts a 10-second timer.
  2. Sends "dispense" to DispenserNode.
* If "dispensed" is received before timeout:

  1. Cancels the initial timer.
  2. Starts a 20-second patient confirmation timer.
* After either timer expires, ReaderNode sends "go" to LineFollowerNode, allowing robot to continue navigation.

---

## Integration with Hardware

The software interfaces with hardware through serial communication:

* *LineFollowerNode:* Reads IR sensors, outputs PWM motor signals via an L298N driver.
* *DispenserNode:* Controls stepper motors through a ULN2003 driver for precise medication dispensing.
* *ReaderNode:* Manages serial communication with both nodes, handles button input via Raspberry Pi GPIO (using pigpio library).

---

## Testing and Validation

Testing and validation procedures include:

* *Unit testing:* Individual node tests via serial monitors and manual inputs.
* *Integration testing:* Combined tests ensuring correct:

  * Robot stopping upon line detection.
  * Medication dispensing triggered via serial commands.
  * Robot resumption after timeouts or patient confirmations.
* *Real-time debugging:* Utilizes ROS log outputs and serial feedback for immediate troubleshooting.

---

## Installation and Execution Instructions

Follow these steps to install and run the software:

### 1. Install ROS (Noetic) on Raspberry Pi

bash
sudo apt install ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash


### 2. Set Up Catkin Workspace

bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash


### 3. Install Python Dependencies

bash
sudo apt install python3-pip python3-serial pigpio
pip3 install rospy


### 4. Make Python Scripts Executable

bash
chmod +x ~/catkin_ws/src/robot_control/scripts/reader_node.py


### 5. Launch ROS Nodes

bash
roslaunch robot_control launch_system.launch
