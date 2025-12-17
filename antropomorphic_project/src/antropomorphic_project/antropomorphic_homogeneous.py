#!/usr/bin/env python3

from sympy import Matrix, cos, sin, pi, Symbol, simplify, trigsimp
from sympy.interactive import printing

class AntropomorphicHomogeneousMatrix(object):

    def __init__(self):

        # To make display prety
        printing.init_printing(use_latex = True)

        theta_i = Symbol("theta_i")
        alpha_i = Symbol("alpha_i")
        r_i = Symbol("r_i")
        d_i = Symbol("d_i")

        DH_Matric_Generic = Matrix([[cos(theta_i), -sin(theta_i)*cos(alpha_i), sin(theta_i)*sin(alpha_i), r_i*cos(theta_i)],
                                    [sin(theta_i), cos(theta_i)*cos(alpha_i), -cos(theta_i)*sin(alpha_i), r_i*sin(theta_i)],
                                    [0, sin(alpha_i), cos(alpha_i), d_i],
                                    [0,0,0,1]])

        result_simpl = simplify(DH_Matric_Generic)

        from sympy import preview

        # Save to local File
        preview(result_simpl, viewer='file', filename="out.png", dvioptions=['-D','300'])


        # Now we create A01, A12, A23

        theta_1 = Symbol("theta_1")
        theta_2 = Symbol("theta_2")
        theta_3 = Symbol("theta_3")


        # WE Set the DH Parameters for the Antropomorphic ARM

        alpha_1 = pi/2.0
        alpha_2 = 0.0
        alpha_3 = 0.0

        # We leave the R values without evaluate to be able to use it afterwards
        # r_1 is 0.0 for sure so, we can evaluate it already
        r_1 = 0.0
        r_2 = Symbol("r_2")
        r_3 = Symbol("r_3")

        d_antropomorf = 0.0
        d_1 = d_antropomorf
        d_2 = d_antropomorf
        d_3 = d_antropomorf

        A01 = DH_Matric_Generic.subs(r_i,r_1).subs(alpha_i,alpha_1).subs(d_i,d_1).subs(theta_i, theta_1)
        A12 = DH_Matric_Generic.subs(r_i,r_2).subs(alpha_i,alpha_2).subs(d_i,d_2).subs(theta_i, theta_2)
        A23 = DH_Matric_Generic.subs(r_i,r_3).subs(alpha_i,alpha_3).subs(d_i,d_3).subs(theta_i, theta_3)

        A03 = A01 * A12 * A23


        A03_simplify = trigsimp(A03)

        # We save

        preview(A03, viewer='file', filename="A03.png", dvioptions=['-D','300'])
        preview(A03_simplify, viewer='file', filename="A03_simplify.png", dvioptions=['-D','300'])


        preview(A01, viewer='file', filename="A01.png", dvioptions=['-D','300'])
        preview(A12, viewer='file', filename="A12.png", dvioptions=['-D','300'])
        preview(A23, viewer='file', filename="A23.png", dvioptions=['-D','300'])

        # We save the object of A03
        self._A03_simplify = A03_simplify

    def get_A03(self):
        return self._A03_simplify