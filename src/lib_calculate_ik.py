#!/usr/bin/env python3

'''
inverse kinematics equations for an antropomorphic arm
---> based on end-effector position 

for links 2 and 3:
basically the solution corresponds to a 2dof planar arm 
for link 1:
it rotates around the z axis, so the solution is simple

in summary, the solution can be respresented as (r,z)
and (r) is the combination of (x,y) 
'''

from math import sin, cos, atan2, sqrt, pi, acos, degrees

def check_theta23(theta2, theta3):
    # theta2 must be between -pi/4 and 3*pi/4
    # theta3 must be between -3*pi/4 and 3*pi/4
    if -pi/4 <= theta2 <= 3*pi/4 and -3*pi/4 <= theta3 <= 3*pi/4:
        return True
    else:
        return False

def normalize_angle(theta):
    # angle theta to be between -pi and pi
    while theta > pi:
        theta = theta - 2*pi
    return theta

def mirror_angle(theta):
    return -theta

def calculate_ik(des_x, des_y, des_z, elbow_config = "down"):
    L1=0 # length of link 1 
    L2=1 # length of link 2
    L3=1 # length of link 3

    r=sqrt(des_x**2 + des_y**2)
    theta1 = atan2(des_y, des_x)
    theta3_plus = acos((r**2 + des_z**2 - L2**2 - L3**2)/(2*L2*L3))
    theta3_nega = -acos((r**2 + des_z**2 - L2**2 - L3**2)/(2*L2*L3))
    theta2_plus = atan2(des_z, r) - atan2(L3*sin(theta3_plus), L2 + L3*cos(theta3_plus))
    theta2_nega = atan2(des_z, r) - atan2(L3*sin(theta3_nega), L2 + L3*cos(theta3_nega))

    if elbow_config == "down":
        print("Angles thetas solved = [",theta1,",",theta2_nega,",",theta3_nega,"], solution is possible: ", check_theta23(theta2_nega, theta3_nega))
        return [theta1, theta2_nega, theta3_nega], check_theta23(theta2_nega, theta3_nega)
    
    else:
        print("Angles thetas solved = [",theta1,",",theta2_plus,",",theta3_plus,"], solution is possible: ", check_theta23(theta2_plus, theta3_plus))
        return [theta1, theta2_plus, theta3_plus], check_theta23(theta2_plus, theta3_plus)
        
if __name__ == "__main__":
    print("Testing Inverse Kinematics")
    print("Test 1")
    print("Desired Position: x=1.0, y=1.0, z=1.0")
    print("Elbow Configuration: down")
    calculate_ik(1.0, 1.0, 1.0, "down")
    print("Test 2")
    print("Desired Position: x=1.0, y=1.0, z=1.0")
    print("Elbow Configuration: up")
    calculate_ik(1.0, 1.0, 1.0, "up")
