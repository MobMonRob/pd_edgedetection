#! /usr/bin/env python

import rospy

import numpy as np

import actionlib

import edge_detection_pmd.msg

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped

class DetectionAction(object):
    # create messages that are used to publish feedback/result
    #_feedback = actionlib_tutorials.msg.FibonacciFeedback()
    _result = edge_detection_pmd.msg.DetectionResult()
    _sum_orientation = Vector3(0,0,0)
    _sum_midpoint = Vector3(0,0,0)

    def __init__(self, name):
        self._action_name = name
	
        self._as = actionlib.SimpleActionServer(self._action_name, edge_detection_pmd.msg.DetectionAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
	self._sum_orientation = Vector3(0,0,0)
	self._sum_midpoint = Vector3(0,0,0)

        r = rospy.Rate(1)
        success = True
	    
        
        
        # publish info to the console for the user
        rospy.loginfo('%s: Executing, Frames %i' % (self._action_name, goal.numberOfFrames))
        
        # start executing the action
        for i in range(0, goal.numberOfFrames):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            rospy.loginfo(i)
	    msg2 = rospy.wait_for_message('object_recognition/position_midpoint', Vector3)
	    rospy.loginfo("Position")
	    rospy.loginfo(msg2)
	    msg = rospy.wait_for_message('object_recognition/orientation', Vector3)
	    rospy.loginfo("Orientation")
	    rospy.loginfo(msg)
	    
	    self._sum_orientation = Vector3(msg.x + self._sum_orientation.x , msg.y + self._sum_orientation.y, msg.z + self._sum_orientation.z)
	    self._sum_midpoint = Vector3(msg2.x + self._sum_midpoint.x, msg2.y + self._sum_midpoint.y, msg2.z + self._sum_midpoint.z)
            # publish the feedback
            #self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()
          
        if success:
	    averaged_orientation = Vector3(self._sum_orientation.x/ goal.numberOfFrames, self._sum_orientation.y/ goal.numberOfFrames, self._sum_orientation.z /goal.numberOfFrames)
	    averaged_midpoint = Vector3(self._sum_midpoint.x/ goal.numberOfFrames, self._sum_midpoint.y/ goal.numberOfFrames, self._sum_midpoint.z /goal.numberOfFrames)
	    rospy.loginfo("got it")
            self._result.orientation = averaged_orientation
	    self._result.midpoint_position = averaged_midpoint
	    self._result.rz = np.arccos(self._sum_orientation.y/1)
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('detection')
    server = DetectionAction(rospy.get_name())
    rospy.spin()

