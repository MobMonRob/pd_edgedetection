#!/usr/bin/env python
from __future__ import print_function

import cv2
import roslib
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3
from Detector import Detector

roslib.load_manifest('edge_detection_pmd')


class Learner(Detector):

    def __init__(self):
        Detector.__init__(self)
        self.state = self.init

        self.object_contour = None
        self.object_gripper_expanse = None
        self.object_center = None
	self.object_rotation = None
	self.pub_position_midpoint = rospy.Publisher("/object_recognition/position_midpoint", PointStamped, queue_size=10)
	self.pub_orientation = rospy.Publisher("/object_recognition/orientation",Vector3, queue_size = 10)
	

    def init(self):
        
        self.state = self.select_object

    def select_object(self):
        # Show colored contours
	
        for contour_index, contour in enumerate(self.contours):
            if contour_index < len(self.colors):
                # drawContours(image, contours, contourIdx, color, thickness)
                self.draw_contour(contour_index, contour_index)
        self.show_image_wait("Learner")

        # Get desired object (Take first element of contours)
        if next(iter(self.contours),None) is not None:
	    self.object_contour = next(iter(self.contours), None)
	    self.state = self.track_object
	#self.object_gripper_expanse = self.get_initial_gripper_expanse(1)
        #for contour_index, contour in enumerate(self.contours):
        #    if contour_index < len(self.colors):
        #        if self.pressed_key == ord(self.colors.keys()[contour_index][0]):
        #             #Saving object details
        #            self.object_contour = contour
        #            #self.object_gripper_expanse = self.get_initial_gripper_expanse(contour_index)
        #            # Inform user
        #            rospy.loginfo("Tracking the selected object.")
        #            rospy.loginfo("Do you want to save it? (y/n)")
        #            self.state = self.track_object

    def track_object(self):
        matching_contours = self.get_matching_contours(self.object_contour)
        if len(matching_contours) > 0:
            best_match = min(matching_contours.items(), key=lambda x: x[1])

            contour_index = best_match[0]
            # Show best result
            # print("Difference:", difference)

            self.draw_contour(contour_index)
            midpoint, finger_point1, finger_point2, rotation, vx, vy, center = self.get_object_parameters(contour_index)
	    self.object_center = midpoint
	    self.object_rotation = rotation
	    self.vx = vx
	    self.vy = vy
 	    self.center3d = center 

        # Get desired action (save/cancel)
        self.show_image_wait("Learner")
	# save object
	#self.state = self.save_object
	rospy.loginfo(self.object_center)
	rospy.loginfo(self.object_rotation)
	rospy.loginfo(self.vx)
	rospy.loginfo(self.vy)
        rospy.loginfo(self.center3d)
	# publish object
	self.publish_object()

        #if self.pressed_key == ord("y"):
         #   self.state = self.save_object
        #if self.pressed_key == ord("n"):
         #   self.state = self.init

    def save_object(self):
	name = "object"
        # Getting object name
        #name = raw_input("Please enter a name for the object...")
        # Saving object
        rospy.loginfo("Saving object under name '" + name + "'...")

	rospy.loginfo(self.object_center)
	rospy.loginfo(self.object_rotation)
	rospy.loginfo(self.vx)
	rospy.loginfo(self.vy)
        rospy.loginfo(self.center3d)
    

	
        self.object_manager.save_object(name, self.object_contour, self.object_gripper_expanse, self.object_center)
        # Back to start
        self.state = self.init

    def publish_object(self):
	self.pub_position_midpoint.publish(PointStamped(self.point_cloud.header, self.center3d))
	self.pub_orientation.publish(Vector3(self.vx, self.vy, 0))


def main():
    Learner()
    rospy.init_node('object_learner', anonymous=True, disable_signals=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
