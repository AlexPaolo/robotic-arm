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

        # angle Physical limits
        self.theta_2_MIN = -pi/4.0
        self.theta_2_MAX = 3*pi/4.0

        self.theta_3_MIN = -3*pi/4.0
        self.theta_3_MAX = 3*pi/4.0


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

        # We declare all the equations for theta1, theta2, theta3 and auxiliary
        #########################################################################
        # theta_3
        C3 = (pow(Pee_x,2) + pow(Pee_y,2) + pow(Pee_z,2) - pow(r2,2) - pow(r3,2)) / (2*r2*r3)

        # We have to chekc that the value insid ethe sqrt is positive
        # otherwise we are working with imaginary numbers
        if pow(C3,2) > 1:
            print("#### ERROR, COSINUS Theta_3 Bigger than 1, no solution")
            return [0.0,0.0,0.0], False
        else:
            if theta_3_config == "plus":
                S3 = sqrt(1-pow(C3,2))
            else:
                S3 = -1*sqrt(1-pow(C3,2))

        # theta_2
        # Auxiliary vars to clean a bit
        if theta_2_config == "plus":
            F = sqrt(pow(Pee_x,2) + pow(Pee_y,2))
        else:
            F = -1*sqrt(pow(Pee_x,2) + pow(Pee_y,2))

        G = pow(r2,2) + pow(r3,2) + 2*r2*r3*C3
        H = (r2 +r3*C3)
        C2 = (F*H + r3*S3*Pee_z) / G
        S2 = (Pee_z*H - F*r3*S3) / G


        # theta_1
        K = r2*C2 + r3*C2*C3 - r3*S2*S3
        S1 = Pee_y / K
        C1 = Pee_x / K


        ## WE HAVE TO CHECK THAT ITS POSSIBLE
        ## -1 <= C3 and S3 <= 1
        ## -1 <= C2 and S2 <= 1
        ## -1 <= C1 and S1 <= 1
        possible_solution = False
        if (C3 <= 1 ) and ( C3>= -1) and (S3 <= 1 ) and ( S3>= -1):
            print("theta_3 solution possible C3=="+str(C3)+", S3="+str(S3))
            if (C2 <= 1 ) and ( C2>= -1) and (S2 <= 1 ) and ( S2>= -1):
                print("theta_2 solution possible C2=="+str(C3)+", S2="+str(S3))
                if (C1 <= 1 ) and ( C1>= -1) and (S1 <= 1 ) and ( S1>= -1):
                    print("theta_1 solution possible C1=="+str(C1)+", S1="+str(S1))
                    possible_solution = True
                else:
                    print("theta_1 solution NOT possible C1=="+str(C1)+", S2="+str(S1))
            else:
                print("theta_2 solution NOT possible C2=="+str(C2)+", S2="+str(S2))
        else:
            print("theta_2 solution NOT possible C3=="+str(C3)+", S3="+str(S3))


        theta_array = []

        if possible_solution:

            theta_3 = atan2(S3, C3)
            theta_2 = atan2(S2*G, C2*G)

            theta_1 = atan2(S1, C1)


            theta_2_possible,theta_3_possible = self.check_phys_limits(theta_2, theta_3)
            possible_solution = theta_2_possible and theta_3_possible


            theta_array = [theta_1, theta_2, theta_3]

        return theta_array, possible_solution


    def check_phys_limits(self,theta_2, theta_3):

        theta_2_possible = True
        theta_3_possible = True

        if theta_2 >self.theta_2_MAX or theta_2 <self.theta_2_MIN:
            theta_2_possible = False
            print(">>>>>>>>>>>>>>> theta_2 NOT POSSIBLE, MIN="+str(self.theta_2_MIN)+", theta_2="+str(theta_2)+", MAX="+str(self.theta_2_MAX))

        if theta_3 >self.theta_3_MAX or theta_3 <self.theta_3_MIN:
            theta_3_possible = False
            print(">>>>>>>>>>>>>>> theta_3 NOT POSSIBLE, MIN="+str(self.theta_3_MIN)+", theta_3="+str(theta_3)+", MAX="+str(self.theta_3_MAX))

        return theta_2_possible,theta_3_possible

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

    Pee_x = -0.5
    Pee_y = 0.5
    Pee_z = -0.5


    calculate_ik(Pee_x, Pee_y, Pee_z, DH_parameters, theta_2_config = "plus", theta_3_config = "plus")
    calculate_ik(Pee_x, Pee_y, Pee_z, DH_parameters, theta_2_config = "plus", theta_3_config = "minus")
    calculate_ik(Pee_x, Pee_y, Pee_z, DH_parameters, theta_2_config = "minus", theta_3_config = "plus")
    calculate_ik(Pee_x, Pee_y, Pee_z, DH_parameters, theta_2_config = "minus", theta_3_config = "minus")