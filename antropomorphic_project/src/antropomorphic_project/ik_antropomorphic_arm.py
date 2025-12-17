#!/usr/bin/env python3

from math import atan2, pi, sin, cos, pow, sqrt



from dataclasses import dataclass
@dataclass
class EndEffectorWorkingSpace:
    # Pos of The rigin of frame_3
    Pee_x: float
    Pee_y: float
    Pee_z: float

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

        # We get all the DH parameters
        r2 = self.get_dh_param("r2")
        r3 = self.get_dh_param("r3")

        print("Input Data===== theta_2_config CONFIG = "+str(theta_2_config))
        print("Input Data===== theta_3_config CONFIG = "+str(theta_3_config))
        print("Pee_x = "+str(Pee_x))
        print("Pee_y = "+str(Pee_y))
        print("Pee_z = "+str(Pee_z))
        print("r2 = "+str(r2))
        print("r3 = "+str(r3))

        ####################
        # Calculate theta_3
        C3 = (pow(Pee_x,2) + pow(Pee_y,2) + pow(Pee_z,2) - pow(r2,2) - pow(r3,2)) / (2*r2*r3)

        if pow(C3,2) > 1:
            print("#### ERROR, COSINUS Theta_3 Bigger than 1, no solution")
            return [0.0,0.0,0.0], False
        else:
            if theta_3_config == "plus":
                S3 = sqrt(1-pow(C3,2))
            else:
                S3 = -1*sqrt(1-pow(C3,2))

        theta_3 = atan2(S3, C3)


        #################################
        # theta_2
        C3_p = abs(C3)
        S3_p = abs(S3)

        H = (r2 +r3*C3_p)
        F = sqrt(pow(Pee_x,2) + pow(Pee_y,2))

        if theta_2_config == "plus" and theta_3_config == "plus":
            S2 = (Pee_z*H - r3*S3_p*F)
            C2 = (H*F + r3*S3_p*Pee_z)

        elif theta_2_config == "minus" and theta_3_config == "plus":
            S2 = (Pee_z*H + r3*S3_p*F)
            C2 = (-1*H*F + r3*S3_p*Pee_z)
        elif theta_2_config == "plus" and theta_3_config == "minus":
            S2 = (Pee_z*H - r3*S3_p*F)
            C2 = (H*F + r3*S3_p*Pee_z)
        elif theta_2_config == "minus" and theta_3_config == "minus":
            S2 = (Pee_z*H + r3*S3_p*F)
            C2 = (-1*H*F + r3*S3_p*Pee_z)
        else:
            assert False, "Combination not supported ="+str(theta_2_config)+","+str(theta_3_config)


        theta_2 = atan2(S2, C2)


        #########################################
        # theta_1
        K = r2*C2 + r3*C2*C3_p - r3*S2*S3_p
        S1 = Pee_y / K
        C1 = Pee_x / K
        theta_1 = atan2(S1, C1)

        # if Pee_y >= 0:
        #     theta_1 = atan2(Pee_y, Pee_x)  - pi
        # else:
        #     theta_1 = atan2(Pee_y, Pee_x)  + pi


        theta_array = []

        # Return values calculated

        theta_array = [theta_1, theta_2, theta_3]

        return theta_array, True

def calculate_ik(Pee_x, Pee_y, Pee_z, DH_parameters, theta_2_config = "plus", theta_3_config = "plus"):


    ik = ComputeIk(DH_parameters = DH_parameters)
    end_effector_pose = EndEffectorWorkingSpace(Pee_x = Pee_x,
                                                Pee_y = Pee_y,
                                                Pee_z = Pee_z)

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

    # Pee_x = 1.0
    # Pee_y = 1.0
    # Pee_z = 1.0

    # Pee_x = 0.6220471703410841
    # Pee_y = 1.3649385765926063
    # Pee_z = 1.0

    Pee_x = 0.5
    Pee_y = 0.5
    Pee_z = 0.1


    calculate_ik(Pee_x, Pee_y, Pee_z, DH_parameters, theta_2_config = "minus", theta_3_config = "minus")