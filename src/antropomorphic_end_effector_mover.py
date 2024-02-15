#!/usr/bin/env python3
import sys
sys.path.append('/home/user/catkin_ws/src/antropomorphic_project/src')
# export PYTHONPATH="${PYTHONPATH}:/home/user/catkin_ws/src/antropomorphic_project/src"

import rospy
from planar_3dof_control.msg import EndEffector
from geometry_msgs.msg import Vector3
import lib_move_joints
import lib_rviz_marker
import lib_calculate_ik

class PlanarEndEffectorMover(object):
    def __init__(self, wait_reach_goal=True):
        # Start the RVIZ marker pblisher
        self.markerbasics_object = lib_rviz_marker.MarkerBasics()
        self.unique_marker_index = 0
        
        # We start the mover
        self.robot_mover = lib_move_joints.JointMover()
        
        # We subscribe to a topic for EE Pose commands
        ee_pose_commands_topic = "/ee_pose_commands"
        rospy.Subscriber(ee_pose_commands_topic, EndEffector, self.ee_pose_commands_callback)
        ee_pose_commands_data = None
        while ee_pose_commands_data is None and not rospy.is_shutdown():
            try:
                ee_pose_commands_data = rospy.wait_for_message(ee_pose_commands_topic, EndEffector, timeout=0.5)
            except:
                rospy.logwarn("Waiting for first EE command Pose in topic =" + str(ee_pose_commands_topic))
                pass
        
        self.Pee_x = ee_pose_commands_data.ee_xy_theta.x
        self.Pee_y = ee_pose_commands_data.ee_xy_theta.y
        self.Pee_z = ee_pose_commands_data.ee_xy_theta.z
        self.elbow_pol = ee_pose_commands_data.elbow_policy.data        

        # We susbcribe to the Real P_3 pose
        end_effector_real_pose_topic = "/end_effector_real_pose"
        rospy.Subscriber(end_effector_real_pose_topic, Vector3, self.end_effector_real_pose_callback)
        end_effector_real_pose_data = None
        while end_effector_real_pose_data is None and not rospy.is_shutdown():
            try:
                end_effector_real_pose_data = rospy.wait_for_message(end_effector_real_pose_topic, Vector3, timeout=0.5)
            except:
                rospy.logwarn("Waiting for first EE command Pose in topic =" + str(end_effector_real_pose_topic))
                pass
        
        self.Pee_x_real = end_effector_real_pose_data.x
        self.Pee_y_real = end_effector_real_pose_data.y
        self.Pee_z_real = end_effector_real_pose_data.z

    def end_effector_real_pose_callback(self,msg):
        self.Pee_x_real = msg.x
        self.Pee_y_real = msg.y
        self.Pee_z_real = msg.z
        rospy.loginfo("Pxx_REAL=["+str(self.Pee_x_real)+","+str(self.Pee_y_real)+","+str(self.Pee_z_real)+"]")
        rospy.loginfo("Pxx_OBJE=["+str(self.Pee_x)+","+str(self.Pee_y)+","+str(self.Pee_z)+"]")

    def ee_pose_commands_callback(self, msg):
        self.Pee_x = msg.ee_xy_theta.x
        self.Pee_y = msg.ee_xy_theta.y
        self.Pee_z = msg.ee_xy_theta.z
        self.elbow_pol = msg.elbow_policy.data
        # calculate the inverse kinematics
        theta_array, possible_solution = lib_calculate_ik.calculate_ik(des_x=self.Pee_x, des_y=self.Pee_y, des_z=self.Pee_z, elbow_config=self.elbow_pol)

        if possible_solution:
            theta_1 = theta_array[0]
            theta_2 = theta_array[1]
            theta_3 = theta_array[2]
            # move the robot to the new pose
            self.robot_mover.move_all_joints(theta_1, theta_2, theta_3)
            self.markerbasics_object.publish_point(self.Pee_x, self.Pee_y, self.Pee_z, index=self.unique_marker_index)
            self.unique_marker_index += 1 
        else:
            rospy.logerr("NO POSSIBLE SOLUTION FOUND, Robot Cant reach that pose")

def main():
    rospy.init_node('planar_end_effector_mover')

    planar_object = PlanarEndEffectorMover()
    rospy.spin()

if __name__ == '__main__':
    main()
