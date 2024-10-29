#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
from pathlib import Path
import numpy as np

class StretchHelloRobotImageCapture:
    def __init__(self):
        self.bridge = CvBridge()  # CvBridge object for converting ROS image to OpenCV format
        rospy.init_node('stretch_hello_image_capture', anonymous=True)

        rospy.loginfo("begin capturing hello robot image...")

        # set up image saving path

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        # self.image_depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.image_depth_callback)
        
        self.image_depth_path = os.path.dirname(os.path.abspath(__file__)) + "/depth_images/"
        self.image_path = os.path.dirname(os.path.abspath(__file__)) + "/images/"
        self.cleanup_image_path()

        self.stored_depth_images = []
        self.stored_images = []
        self.image_index = 0
        # self.image_sub = rospy.Subscriber('/gazebo/default/user_camera/pose', Image, self.image_callback)

    def cleanup_image_path(self):
        image_paths = [self.image_path, self.image_depth_path]
        for img_path in image_paths:
            if not os.path.exists(img_path):
                os.makedirs(img_path)
            else:
                for path in Path(img_path).glob("**/*"):
                    if path.is_file():
                        path.unlink()
                        
                rospy.loginfo("Clear up the image folder:" + img_path)


    def image_depth_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV format
            depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="32FC1")
        except CvBridgeError as e:
            rospy.logerr(f"Could not convert depth image: {e}")
            return

        # Process the depth image (example: normalize and visualize)
        depth_array = np.array(depth_image, dtype=np.float32)
        depth_array = np.nan_to_num(depth_array)  # Replace NaNs with 0

        # Normalize for visualization purposes (optional)
        normalized_depth = cv2.normalize(depth_array, None, 0, 255, cv2.NORM_MINMAX)
        depth_colormap = cv2.applyColorMap(normalized_depth.astype(np.uint8), cv2.COLORMAP_JET)
       
        # Display the depth image
        cv2.imshow("Depth Image", depth_colormap)
        cv2.waitKey(1)
    
        # Save the image (you can modify the path as needed)
        depth_image_name = "stretch_hello_robot_image_depth_" + str(self.image_index) + ".jpg"
        cv2.imwrite(self.image_depth_path, depth_colormap)

        self.stored_depth_images.append(depth_image_name)
        if len(self.stored_depth_images) > 20:
            os.remove(self.image_depth_path + self.stored_depth_images.pop(0))

        rospy.loginfo("Image saved to %s", self.image_depth_path)


    def image_callback(self, data):
        try:
            # Convert the ROS Image message to a format OpenCV can use
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
           
            # Show image in a window (you can comment this out if running in a headless environment)
            cv2.imshow("Stretch Hello Robot Camera", cv_image)
            cv2.waitKey(1)  # Add delay for image display, can be 0 for wait forever
           
            # Save the image (you can modify the path as needed)
            image_name = "stretch_hello_robot_image_" + str(self.image_index) + ".jpg"
            image_save_path = self.image_path + image_name
            cv2.imwrite(image_save_path, cv_image)

            self.image_index += 1
            self.stored_images.append(image_name)
            if len(self.stored_images) > 20:
                os.remove(self.image_path + self.stored_images.pop(0))

            rospy.loginfo("Image saved to %s", image_save_path)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

if __name__ == '__main__':
    try:

        image_capture_obj = StretchHelloRobotImageCapture()
        rospy.spin()  # Keep the node running until shutdown
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()  # Close any OpenCV windows when the node shuts down