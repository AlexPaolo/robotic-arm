#!/usr/bin/env python3

from math import atan2, pi, sin, cos, pow, sqrt



from dataclasses import dataclass
@dataclass
class EndEffectorWorkingSpace:
    # Pos of The rigin of frame_3
    Pee_x: float
    Pee_y: float
    Pee_z: float
    # Orientation of the X axis of frame_3 respect frame_0
    Xee_x: float
    Xee_y: float
    Xee_z: float

class ComputeIk():

    def __init__(self, DH_parameters):

        # DH parameters
        self.DH_parameters_ = DH_parameters

    def get_dh_param(self, name):

        if name in self.DH_parameters_:
            return self.DH_parameters_[name]
        else:
            assert False, "Asked for Non existen param DH name ="+str(name)

    def compute_ik(self, end_effector_pose, theta_2_config, theta_3_config):

        # Initialization
        Pee_x = end_effector_pose.Pee_x
        Pee_y = end_effector_pose.Pee_y
        Pee_z = end_effector_pose.Pee_z

        # The Angle that Xee makes with frames 0 x axis
        Xee_x = end_effector_pose.Xee_x
        Xee_y = end_effector_pose.Xee_y
        Xee_z = end_effector_pose.Xee_z

        # We get all the DH parameters
        r2 = self.get_dh_param("r2")
        r3 = self.get_dh_param("r3")

        print("Input Data===== theta_2_config CONFIG = "+str(theta_2_config))
        print("Input Data===== theta_3_config CONFIG = "+str(theta_3_config))
        print("Pee_x = "+str(Pee_x))
        print("Pee_y = "+str(Pee_y))
        print("Pee_z = "+str(Pee_z))
        print("Xee_x = "+str(Xee_x))
        print("Xee_y = "+str(Xee_y))
        print("Xee_z = "+str(Xee_z))
        print("r2 = "+str(r2))
        print("r3 = "+str(r3))

        # We declare all the equations for theta1, theta2, theta3 and auxiliary
        #########################################################################
        # theta_2
        U = (pow(Pee_x,2) + pow(Pee_y,2) + pow(Pee_z,2) - pow(r2,2) - pow(r3,2)) / (2*r2*r3)
        W = (Pee_z - r3*Xee_z) / r2


        ## WE HAVE TO CHECK THAT ITS POSSIBLE
        ## -1 <= U <= 1
        possible_solution = False
        if (U <= 1 ) and ( U>= -1):
            print("U value possible=="+str(U))
            if (W <= 1 ) and ( W>= -1):
                print("W value possible=="+str(W))
                possible_solution = True
            else:
                print("W value NOT possible=="+str(W))
        else:
            print("U value NOT possible=="+str(U))


        theta_array = []

        if possible_solution:

            theta_1 = atan2(Xee_y, Xee_x)
            #########################################################################


            #########################################################################

            # We have to decide which solution we want
            if theta_2_config == "plus":
                # Positive
                denominator_2 = sqrt(1-pow(W,2))
            else:
                denominator_2 = -1.0 * sqrt(1-pow(W,2))

            numerator_2 = W

            theta_2 = atan2(numerator_2, denominator_2)
            #########################################################################


            #########################################################################
            # We have to decide which solution we want
            if theta_3_config == "plus":
                # Positive
                numerator_3 = sqrt(1-pow(U,2))
            else:
                numerator_3 = -1.0 * sqrt(1-pow(U,2))

            denominator_3 = U

            theta_3 = atan2(numerator_3, denominator_3)

            #########################################################################

            theta_array = [theta_1, theta_2, theta_3]

        return theta_array, possible_solution

def calculate_ik(Pee_x, Pee_y, Pee_z, beta_pitch_angle, alpha_yaw_angle, DH_parameters, theta_2_config = "plus", theta_3_config = "plus"):

    # We extract the formulas form the rotation_matrix_antropomorfic.py output
    # Xee_x = cos(alpha_yaw_angle)*cos(beta_pitch_angle)
    # Xee_y = sin(alpha_yaw_angle)*cos(beta_pitch_angle)
    # Xee_z = -sin(beta_pitch_angle)

    Xee_x = cos(alpha_yaw_angle)*cos(beta_pitch_angle)
    Xee_y = sin(alpha_yaw_angle)*cos(beta_pitch_angle)
    Xee_z = -sin(beta_pitch_angle)

    print("")

    ik = ComputeIk(DH_parameters = DH_parameters)
    end_effector_pose = EndEffectorWorkingSpace(Pee_x = Pee_x,
                                                Pee_y = Pee_y,
                                                Pee_z = Pee_z,
                                                Xee_x = Xee_x,
                                                Xee_y = Xee_y,
                                                Xee_z = Xee_z)

    thetas, possible_solution = ik.compute_ik(  end_effector_pose=end_effector_pose,
                                                theta_2_config = theta_2_config,
                                                theta_3_config = theta_3_config)

    print("Angles thetas solved ="+str(thetas))
    print("possible_solution = "+str(possible_solution))

    return thetas, possible_solution

if __name__ == '__main__':


 
    r2 = 1.0
    r3 = 1.0

    # theta_i here are valriables of the joints
    # We only fill the ones we use in the equations, the others were already
    # replaced in the Homogeneous matrix
    DH_parameters={ "r2":r2,
                    "r3":r3}

    # Pee_x = 2.0
    # Pee_y = 0.0
    # Pee_z = 0.0
    # beta_pitch_angle = 0.0
    # alpha_yaw_angle = 0.0

    # calculate_ik(Pee_x, Pee_y, Pee_z, beta_pitch_angle, alpha_yaw_angle, DH_parameters, theta_2_config = "plus", theta_3_config = "plus")

    # Pee_x = 1.529
    # Pee_y = 1.288
    # Pee_z = -0.041
    # beta_pitch_angle = 0.0
    # alpha_yaw_angle = 0.7

    # calculate_ik(Pee_x, Pee_y, Pee_z, beta_pitch_angle, alpha_yaw_angle, DH_parameters, theta_2_config = "plus", theta_3_config = "plus")

    # Pee_x = 1.702
    # Pee_y = 0.0
    # Pee_z = -0.333
    # beta_pitch_angle = -0.328
    # alpha_yaw_angle = 0.0

    # calculate_ik(Pee_x, Pee_y, Pee_z, beta_pitch_angle, alpha_yaw_angle, DH_parameters, theta_2_config = "plus", theta_3_config = "plus")


    Pee_x = 2.0
    Pee_y = 0.0
    Pee_z = 0.0
    beta_pitch_angle = pi/4
    alpha_yaw_angle = 0.0

    calculate_ik(Pee_x, Pee_y, Pee_z, beta_pitch_angle, alpha_yaw_angle, DH_parameters, theta_2_config = "plus", theta_3_config = "plus")
    # calculate_ik(Pee_x, Pee_y, Pee_z, beta_pitch_angle, alpha_yaw_angle, DH_parameters, theta_2_config = "plus", theta_3_config = "min")
    # calculate_ik(Pee_x, Pee_y, Pee_z, beta_pitch_angle, alpha_yaw_angle, DH_parameters, theta_2_config = "min", theta_3_config = "plus")
    # calculate_ik(Pee_x, Pee_y, Pee_z, beta_pitch_angle, alpha_yaw_angle, DH_parameters, theta_2_config = "min", theta_3_config = "min")

