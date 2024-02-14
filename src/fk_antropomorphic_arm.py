#!/usr/bin/env python3

from sympy import Matrix, cos, sin, Symbol, simplify, trigsimp, pi
from sympy.interactive import printing

# To make display prety
printing.init_printing(use_latex = True)

theta_i = Symbol("theta_i")
alpha_i = Symbol("alpha_i")
r_i = Symbol("r_i")
d_i = Symbol("d_i")

# Geometry of the robot
lenght_link_1 = 0.0
lenght_link_2 = 1.0
lenght_link_3 = 1.0

DH_Matric_Generic = Matrix([[cos(theta_i), -sin(theta_i)*cos(alpha_i), sin(theta_i)*sin(alpha_i), r_i*cos(theta_i)],
                            [sin(theta_i), cos(theta_i)*cos(alpha_i), -cos(theta_i)*sin(alpha_i), r_i*sin(theta_i)],
                            [0, sin(alpha_i), cos(alpha_i), d_i],
                            [0,0,0,1]])

result_simpl = simplify(DH_Matric_Generic)

from sympy import preview

# Save to local file
# preview(result_simpl, viewer='file', filename="denavit-hartenberg.png", dvioptions=['-D','300'])


# Now create A01, A12, A23

theta_1 = Symbol("theta_1")
theta_2 = Symbol("theta_2")
theta_3 = Symbol("theta_3")


alpha_planar = 0.0

alpha_1 = pi/2
alpha_2 = alpha_planar
alpha_3 = alpha_planar

r_colinear = 0.0
r_1 = r_colinear # r_colinear
r_2 = lenght_link_2 # Symbol("r_2") # length of link 2
r_3 = lenght_link_3 # Symbol("r_3") # length of link 3

d_planar = 0.0
d_1 = lenght_link_1 # length of link 1
d_2 = d_planar
d_3 = d_planar

A01 = DH_Matric_Generic.subs(r_i,r_1).subs(alpha_i,alpha_1).subs(d_i,d_1).subs(theta_i, theta_1)
A12 = DH_Matric_Generic.subs(r_i,r_2).subs(alpha_i,alpha_2).subs(d_i,d_2).subs(theta_i, theta_2)
A23 = DH_Matric_Generic.subs(r_i,r_3).subs(alpha_i,alpha_3).subs(d_i,d_3).subs(theta_i, theta_3)

A03 = A01 * A12 * A23
A02 = A01 * A12

A03_simplify = trigsimp(A03)
A02_simplify = trigsimp(A02)

# We save

# preview(A03, viewer='file', filename="A03.png", dvioptions=['-D','300'])
# preview(A02, viewer='file', filename="A02.png", dvioptions=['-D','300'])

# preview(A01, viewer='file', filename="A01.png", dvioptions=['-D','300'])
# preview(A12, viewer='file', filename="A12.png", dvioptions=['-D','300'])
# preview(A23, viewer='file', filename="A23.png", dvioptions=['-D','300'])
# preview(A03_simplify, viewer='file', filename="A03_simplify.png", dvioptions=['-D','300'])
# preview(A02_simplify, viewer='file', filename="A02_simplify.png", dvioptions=['-D','300'])

# given theta1,2,3, provides the position and orientation of the Frame 3
input_tetha_1 = input("Enter the value for theta_1: ")
input_tetha_2 = input("Enter the value for theta_2: ")
input_tetha_3 = input("Enter the value for theta_3: ")

A03_evaluated = A03_simplify.subs(theta_1, input_tetha_1).subs(theta_2, input_tetha_2).subs(theta_3, input_tetha_3)
preview(A03_evaluated, viewer='file', filename="A03_simplify_evaluated.png", dvioptions=['-D','300'])

print("Position Matrix:", A03_evaluated[0:3,3])
print("Orientation Matrix:", A03_evaluated[0:3,0:3])