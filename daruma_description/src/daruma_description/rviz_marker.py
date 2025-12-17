#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from math import sin, cos, tan, pi
import tf
from tf.transformations import quaternion_from_euler

class MarkerBasics(object):

    def __init__(self):
        self.marker_objectlisher = rospy.Publisher('/rigid_body', Marker, queue_size=1)

        self.init_marker(index=0)

        # We start the Tf broadcaster
        self._broad_caster_tf = tf.TransformBroadcaster()

    def init_marker(self,index=0):
        self.marker_object = Marker()
        self.marker_object.header.frame_id = "world"
        self.marker_object.header.stamp    = rospy.get_rostime()
        self.marker_object.ns = ""
        self.marker_object.id = index
        self.marker_object.type = Marker.MESH_RESOURCE
        self.marker_object.action = Marker.ADD

        my_point = Point()
        self.marker_object.pose.position = my_point

        self.marker_object.pose.orientation.x = 0
        self.marker_object.pose.orientation.y = 0
        self.marker_object.pose.orientation.z = 0.0
        self.marker_object.pose.orientation.w = 1.0

        self.marker_object.scale.x = 1.0
        self.marker_object.scale.y = 1.0
        self.marker_object.scale.z = 1.0

        self.marker_object.color.r = 0.0
        self.marker_object.color.g = 0.0
        self.marker_object.color.b = 0.0
        # This has to be, otherwise it will be transparent
        self.marker_object.color.a = 1.0
        # Mesh related data
        self.marker_object.mesh_resource = "package://daruma_description/meshes/daruma.dae";
        self.marker_object.mesh_use_embedded_materials = True

        # If we want it for ever, 0, otherwise seconds before desapearing
        self.marker_object.lifetime = rospy.Duration(0)




    def handle_tf_pose(self, robot_name="daruma", reference_frame_data="/world"):

        self._broad_caster_tf.sendTransform((   self.marker_object.pose.position.x,
                                                self.marker_object.pose.position.y,
                                                self.marker_object.pose.position.z),
                                            (   self.marker_object.pose.orientation.x,
                                                self.marker_object.pose.orientation.y,
                                                self.marker_object.pose.orientation.z,
                                                self.marker_object.pose.orientation.w),
                                            rospy.Time.now(),
                                            robot_name,
                                            reference_frame_data)

    def update_pose(self, x, y, z, roll, pitch, yaw, index, scale_x, scale_y, scale_z):
        """
        roll = Rotation around Z axis
        pitch = rotation around Y axis
        yaw, rotation around x axis
        """

        self.marker_object.id = index

        my_point = Point()

        my_point.x = x
        my_point.y = y
        my_point.z = z
        self.marker_object.pose.position = my_point

        #q = euler_to_quaternion(roll=roll, pitch=pitch, yaw=yaw)
        q = quaternion_from_euler(roll, pitch, yaw)

        self.marker_object.pose.orientation.x = q[0]
        self.marker_object.pose.orientation.y = q[1]
        self.marker_object.pose.orientation.z = q[2]
        self.marker_object.pose.orientation.w = q[3]

        self.marker_object.scale.x = scale_x
        self.marker_object.scale.y = scale_y
        self.marker_object.scale.z = scale_z

        self.handle_tf_pose(robot_name="daruma", reference_frame_data="world")


    def publish_point(self,x, y, z, roll, pitch, yaw,  index=0, scale_x=1.0, scale_y=1.0, scale_z=1.0):

        self.update_pose(x, y, z, roll, pitch, yaw, index, scale_x=scale_x, scale_y=scale_y, scale_z=scale_z)
        self.marker_objectlisher.publish(self.marker_object)

    def start(self):
        rate = rospy.Rate(10)
        theta = 0.0
        index = 0
        scale_x = 1.0
        scale_y = 1.0
        scale_z = 1.0
        while not rospy.is_shutdown():
            theta = 0
            self.publish_point(cos(theta), sin(theta), 0,roll=0, pitch=0, yaw=theta, index=index, scale_x=scale_x, scale_y=scale_y, scale_z=scale_z)
            theta += 0.1

            if theta >= 2*pi:
                theta = 0


            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('marker_basic_node', anonymous=True)
    markerbasics_object = MarkerBasics()
    try:
        markerbasics_object.start()
    except rospy.ROSInterruptException:
        pass