#!/usr/bin/env python3

import rospy
from task_handler.srv import GetObjects

if __name__ == "__main__":
    rospy.init_node("stretch_identify_object_client")
    rospy.wait_for_service("/get_objects") 

    try:
        rospy.loginfo("waiting to receive data:")
        process_data = rospy.ServiceProxy("/get_objects", GetObjects)
        response = process_data()
        rospy.loginfo(f"Received data. {response.objects}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")