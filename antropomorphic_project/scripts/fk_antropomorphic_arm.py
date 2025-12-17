#!/usr/bin/env python3

from math import pi
from sympy import Symbol
from sympy import preview
from antropomorphic_project.antropomorphic_homogeneous import AntropomorphicHomogeneousMatrix


homogeneous_mattrix_obj = AntropomorphicHomogeneousMatrix()
A03_simplify = homogeneous_mattrix_obj.get_A03()

##############
# THETA VALUES
##############
theta_1 = Symbol("theta_1")
theta_2 = Symbol("theta_2")
theta_3 = Symbol("theta_3")

theta_1_val = pi/6
theta_2_val = pi/6
theta_3_val = pi/6

r_1 = 0.0
r_2 = Symbol("r_2")
r_3 = Symbol("r_3")

r_1_val = 0.0
r_2_val = 1.0
r_3_val = 1.0

A03_simplify_evaluated = A03_simplify.subs(theta_1,theta_1_val).subs(theta_2,theta_2_val).subs(theta_3, theta_3_val).subs(r_1, r_1_val).su
bs(r_2, r_2_val).subs(r_3, r_3_val)

# We save
preview(A03_simplify_evaluated, viewer='file', filename="A03_simplify_evaluated.png", dvioptions=['-D','300'])