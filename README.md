# Arm Kinematics - ROS1

The main objectives of this project are:
- Find the Denavit Hartenberg parameters of the robot
- Do the forward kinematics
- Do the inverse kinematics

---
## Setup the project

### Launch simulation

    source ~/simulation_ws/devel/setup.bash
    roslaunch antropomorphic_arm_gazebo main.launch

### Launch rviz

    rosrun rviz rviz -d /home/user/catkin_ws/src/antropomorphic_3dof.rviz

### Launch Control GUI

    roslaunch antropomorphic_arm_control antropomorphic_arm_sim_rqt.launch 

### For check the position

    rostopic echo /end_effector_real_pose

---
## Execute Tasks
### Task 1

    cd ~/catkin_ws
    rosrun antropomorphic_project generate_matrixes.py

### Task 2

    rosrun antropomorphic_project fk_antropomorphic_arm.py

### Task 3

    rosrun antropomorphic_project ik_antropomorphic_arm.py

### Task 4

    roslaunch antropomorphic_project start_elipsoidal_motion.launch
