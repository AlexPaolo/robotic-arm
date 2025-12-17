#!/usr/bin/env python3
import rospy
from planar_3dof_control.msg import EndEffector
from geometry_msgs.msg import Vector3
from antropomorphic_project.ik_antropomorphic_arm_myremake import calculate_ik
from antropomorphic_project.move import JointMover
from antropomorphic_project.rviz_marker import MarkerBasics

class PlanarEndEffectorMover(object):

    def __init__(self, wait_reach_goal=True):

        # Start the RVIZ marker pblisher
        self.markerbasics_object = MarkerBasics()
        self.unique_marker_index = 0

        # We start the mover
        self.robot_mover = JointMover()

        # We subscribe to a topic for EE Pose commands

        ee_pose_commands_topic = "/ee_pose_commands"
        rospy.Subscriber(ee_pose_commands_topic, EndEffector, self.ee_pose_commands_clb)
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
        rospy.Subscriber(end_effector_real_pose_topic, Vector3, self.end_effector_real_pose_clb)
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

    def end_effector_real_pose_clb(self,msg):

        self.Pee_x_real = msg.x
        self.Pee_y_real = msg.y
        self.Pee_z_real = msg.z

        rospy.loginfo("Pxx_REAL=["+str(self.Pee_x_real)+","+str(self.Pee_y_real)+","+str(self.Pee_z_real)+"]")
        rospy.loginfo("Pxx_OBJE=["+str(self.Pee_x)+","+str(self.Pee_y)+","+str(self.Pee_z)+"]")

    def ee_pose_commands_clb(self, msg):

        self.Pee_x = msg.ee_xy_theta.x
        self.Pee_y = msg.ee_xy_theta.y
        self.Pee_z = msg.ee_xy_theta.z

        self.elbow_pol = msg.elbow_policy.data


        r2 = 1.0
        r3 = 1.0

        DH_parameters={ "r2":r2,
                        "r3":r3}

        # We parse for the two joints
        theta_2_config = self.elbow_pol.split("-")[0]
        theta_3_config = self.elbow_pol.split("-")[1]

        theta_array, possible_solution = calculate_ik(Pee_x=self.Pee_x, Pee_y=self.Pee_y, Pee_z=self.Pee_z, DH_parameters=DH_parameters, theta_2_config = theta_2_config, theta_3_config = theta_3_config)

        if possible_solution:

            rospy.logwarn("FOUND DIRECT SOLUTION!")


            theta_1 = theta_array[0]
            theta_2 = theta_array[1]
            theta_3 = theta_array[2]

            self.move_joints(theta_1, theta_2, theta_3)

        else:
            # We now try the other options:
            rospy.logwarn("SERCHING FOR ALTERNATIVE SOLUTIONS...............")
            config_array = [["plus","plus"],["minus","plus"],["minus","minus"]]

            for configs in config_array:
                theta_2_config = configs[0]
                theta_3_config = configs[1]

                theta_array, possible_solution = calculate_ik(Pee_x=self.Pee_x,
                                                            Pee_y=self.Pee_y,
                                                            Pee_z=self.Pee_z,
                                                            DH_parameters=DH_parameters,
                                                            theta_2_config = theta_2_config,
                                                            theta_3_config = theta_3_config)

                if possible_solution:
                    rospy.logwarn("FOUND ALTERNATIVE SOLUTION")
                    break

            if possible_solution:
                theta_1 = theta_array[0]
                theta_2 = theta_array[1]
                theta_3 = theta_array[2]

                self.move_joints(theta_1, theta_2, theta_3)
            else:
                rospy.logerr("NO POSSIBLE SOLUTION FOUND, Robot Cant reach that pose")


    def move_joints(self,theta_1, theta_2, theta_3):
        self.robot_mover.move_all_joints(theta_1, theta_2, theta_3)
        self.markerbasics_object.publish_point(self.Pee_x, self.Pee_y, self.Pee_z, index=self.unique_marker_index)
        self.unique_marker_index += 1

def main():
    rospy.init_node('planar_end_effector_mover')

    planar_object = PlanarEndEffectorMover()
    rospy.spin()



if __name__ == '__main__':
    main()