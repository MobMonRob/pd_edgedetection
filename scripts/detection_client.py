#! /usr/bin/env python
from __future__ import print_function
import rospy


import actionlib

import edge_detection_pmd.msg

def detection_client():
    client = actionlib.SimpleActionClient("detection", edge_detection_pmd.msg.DetectionAction)
    client.wait_for_server()
    goal = edge_detection_pmd.msg.DetectionGoal(numberOfFrames = 1)
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('detection_client')
        result = detection_client()
	print("Orientation")
	print(result.orientation)
	print("Midpoint")
	print(result.midpoint_position)
        
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file = sys.stderr)
    
