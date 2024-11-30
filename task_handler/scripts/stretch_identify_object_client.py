#!/usr/bin/env python3

import rospy
from task_handler.srv import GetObjects
from skimage import io

if __name__ == "__main__":
    rospy.init_node("stretch_identify_object_client")

    try:
        rospy.wait_for_service("/get_objects", timeout=4.0) 
        rospy.loginfo("waiting to receive data:")
        process_data = rospy.ServiceProxy("/get_objects", GetObjects)
        response = process_data()
        rospy.loginfo(f"Received data. {response.objects}, {response.real_objects}, {response.output_file}")
        img = io.imread(response.output_file)
        io.imshow(img)
        io.show()
        
    except (rospy.ServiceException, rospy.exceptions) as e:
        rospy.logerr(f"Service call failed: {e}")