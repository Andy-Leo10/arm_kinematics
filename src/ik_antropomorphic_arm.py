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

L1=0 # length of link 1 
L2=1 # length of link 2
L3=1 # length of link 3

des_x = float(input("Enter the x coordinate of the end effector: "))
des_y = float(input("Enter the y coordinate of the end effector: "))
des_z = float(input("Enter the z coordinate of the end effector: "))

r=sqrt(des_x**2 + des_y**2)
theta1 = atan2(des_y, des_x)
theta3_plus = acos((r**2 + des_z**2 - L2**2 - L3**2)/(2*L2*L3))
theta3_nega = -acos((r**2 + des_z**2 - L2**2 - L3**2)/(2*L2*L3))
theta2_plus = atan2(des_z, r) - atan2(L3*sin(theta3_plus), L2 + L3*cos(theta3_plus))
theta2_nega = atan2(des_z, r) - atan2(L3*sin(theta3_nega), L2 + L3*cos(theta3_nega))

# print the 2 results that are possible
print("Angles thetas solved = [",theta1,",",theta2_plus,",",theta3_plus,"], solution is possible: ", check_theta23(theta2_plus, theta3_plus))
print("Angles thetas solved = [",theta1,",",theta2_nega,",",theta3_nega,"], solution is possible: ", check_theta23(theta2_nega, theta3_nega))
# print("Angles thetas solved = [",degrees(theta1),",",degrees(theta2_plus),",",degrees(theta3_plus),"], solution is possible: ", check_theta23(theta2_plus, theta3_plus))
# print("Angles thetas solved = [",degrees(theta1),",",degrees(theta2_nega),",",degrees(theta3_nega),"], solution is possible: ", check_theta23(theta2_nega, theta3_nega))

# BUT rememeber that there are 2 extra possible solutions if theta1 is increased by pi
theta1 = theta1 + pi
theta1 = normalize_angle(theta1)
 
# mirror the angles around pi/2 theta2 and theta3 and limit to [-pi, pi]
theta2_plus_2 = -pi + abs(theta2_plus)
theta2_plus_2 = normalize_angle(theta2_plus_2)
theta3_plus_2 = -theta3_plus
theta3_plus_2 = normalize_angle(theta3_plus_2)

theta2_nega_2 = -pi + abs(theta2_nega)
theta2_nega_2 = normalize_angle(theta2_nega_2)
theta2_nega_2 = mirror_angle(theta2_nega_2)
theta3_nega_2 = -theta3_nega
theta3_nega_2 = normalize_angle(theta3_nega_2)

# print the 2 results that are possible
print("Angles thetas solved = [",theta1,",",theta2_plus_2,",",theta3_plus_2,"], solution is possible: ", check_theta23(theta2_plus_2, theta3_plus_2))
print("Angles thetas solved = [",theta1,",",theta2_nega_2,",",theta3_nega_2,"], solution is possible: ", check_theta23(theta2_nega_2, theta3_nega_2))
# print("Angles thetas solved = [",degrees(theta1),",",degrees(theta2_plus_2),",",degrees(theta3_plus_2),"], solution is possible: ", check_theta23(theta2_plus_2, theta3_plus_2))
# print("Angles thetas solved = [",degrees(theta1),",",degrees(theta2_nega_2),",",degrees(theta3_nega_2),"], solution is possible: ", check_theta23(theta2_nega_2, theta3_nega_2))

# theta2_plus_2 = 2*pi - theta2_plus
# # angle theta2 to be between -pi and pi
# while theta2_plus_2 > pi:
#     theta2_plus_2 = theta2_plus_2 - 2*pi
# theta3_plus_2 = -theta3_plus

# theta2_nega_2 = 2*pi - theta2_nega
# # angle theta2 to be between -pi and pi
# while theta2_nega_2 > pi:
#     theta2_nega_2 = theta2_nega_2 - 2*pi
# theta3_nega_2 = -theta3_nega