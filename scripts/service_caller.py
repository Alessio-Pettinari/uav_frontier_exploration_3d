#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool 

def call_services():
    rospy.wait_for_service('/first/exploration/toggle')
    rospy.wait_for_service('/drone/exploration/toggle')
    
    try:
        # Chiamata al primo servizio
        first_service_client = rospy.ServiceProxy('/first/exploration/toggle', SetBool)
        response_first = first_service_client(True) # call with "data:True"
        rospy.loginfo("First service response: %s", response_first.message)
        
        # Chiamata al secondo servizio
        drone_service_client = rospy.ServiceProxy('/drone/exploration/toggle', SetBool)
        response_drone = drone_service_client(True) # call with "data:True"
        rospy.loginfo("Drone service response: %s", response_drone.message)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    rospy.init_node('service_caller')
    call_services()
