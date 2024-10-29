#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
from pathlib import Path
from task_handler.msg import Object, Objects
from geometry_msgs.msg import Point, Quaternion, Pose
# import stretch_ros.stretch_deep_perception.nodes.object_detector as jd
# import stretch_ros.stretch_deep_perception.nodes.deep_learning_model_options as do
# import glob
import sys

# def process_image():
    
#     models_directory = do.get_directory()
#     print('Using the following directory for deep learning models:', models_directory)        
#     use_neural_compute_stick = do.use_neural_compute_stick()
#     if use_neural_compute_stick:
#         print('Attempt to use an Intel Neural Compute Stick 2.')
#     else:
#         print('Not attempting to use an Intel Neural Compute Stick 2.')
  
#     only_display_result_images = True

#     input_dir = imageSaveDir
#     output_dir = os.path.dirname(os.path.abspath(__file__)) + "/out_images"
#     filenames = glob.glob(input_dir + '*')
#     filenames.sort()

#     print('Will attempt to load the following files:')
#     for f in filenames:
#         print(f)

#     use_tiny = True
#     if use_tiny:
#         confidence_threshold = 0.0
#     else:
#         confidence_threshold = 0.5
        
#     object_detector = jd.ObjectDetector(models_directory,
#                                         use_tiny_yolo3=use_tiny,
#                                         use_neural_compute_stick=use_neural_compute_stick)
    
#     for i, f in enumerate(filenames): 
#         rgb_image = cv2.imread(f)
#         if rgb_image is not None:
#             output_image = rgb_image.copy()
#             results, output_image = object_detector.apply_to_image(rgb_image, draw_output=True)
#             out_filename = output_dir + 'object_detection_' + str(i) + '.png'
#             print('writing', out_filename)
#             cv2.imwrite(out_filename, output_image)



if __name__ == '__main__':
    try:
        # set up image saving path
        imageSaveDir = os.path.dirname(os.path.abspath(__file__)) + "/images"

        rospy.init_node('stretch_hello_obj_loc_publisher', )
        rospy.loginfo("begin sending object detection info...")
        obj_pub = rospy.Publisher('/object_loc', Objects, queue_size=10)
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            # process_image()
            msg = Objects()
            obj_list = []

            obj = Object()
            obj.name = "ball"
            obj.pose = Pose(Point(12.0, 24.0, 18.0), Quaternion(1.2, 3.2, 4.2, 5.0))
            obj_list.append(obj)

            obj = Object()
            obj.name = "rubbish_bin"
            obj.pose = Pose(Point(12.0, 24.0, 18.0), Quaternion(1.2, 3.2, 4.2, 5.0))
            obj_list.append(obj)
            msg.objects = obj_list
            obj_pub.publish(msg)

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows() 

        
    # imageSaveDir = os.path.dirname(os.path.abspath(__file__)) + "/images"
    # process_image()