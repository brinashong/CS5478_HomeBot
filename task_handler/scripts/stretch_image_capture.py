#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
from pathlib import Path
import numpy as np
import message_filters


class ImageSynchronizer:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('stretch_image_capture', anonymous=True)
        rospy.loginfo("begin capturing robot image...")

        # set up image saving path
        self.image_depth_path = os.path.dirname(os.path.abspath(__file__)) + "/depth_images/"
        self.image_path = os.path.dirname(os.path.abspath(__file__)) + "/images/"
        self.cleanup_image_path()

        # self.stored_depth_images = []
        self.stored_images_index = []
        self.image_index = 0

        # Subscribers for color and depth images
        self.color_sub = message_filters.Subscriber("camera/color/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("camera/depth/image_raw", Image)

        # Time synchronizer with a 0.1-second tolerance window
        self.sync = message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=1000, slop=0.1)
        self.sync.registerCallback(self.callback)

    def callback(self, color_msg, depth_msg):
        try:
            # Convert messages to OpenCV images
            color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")  # Assuming depth is in meters
        
            # # optional: save colormap
            # # normalized_depth = cv2.normalize(depth_array, None, 0, 255, cv2.NORM_MINMAX)
            # # depth_colormap = cv2.applyColorMap(normalized_depth.astype(np.uint8), cv2.COLORMAP_JET)
            # depth_colormap = cv2.applyColorMap(depth_array.astype(np.uint8), cv2.COLORMAP_JET)

            # Save the image (you can modify the path as needed)
            depth_image_name = "image_depth_" + str(self.image_index) + ".png"
            image_name = "image_" + str(self.image_index) + ".png"

            self.stored_images_index.append(self.image_index)

            # restrict the total number of files in the images and depth_images folders
            if len(self.stored_images_index) > 500:
                oldest_file_index  = self.stored_images_index[0]
                os.remove(self.image_path + "image_" + str(oldest_file_index) + ".png")
                os.remove(self.image_depth_path + "image_depth_" + str(oldest_file_index) + ".png")
                self.stored_images_index.pop(0)

            cv2.imwrite(self.image_depth_path + depth_image_name, depth_image)
            # cv2.imwrite(self.image_depth_path + depth_image_name, depth_colormap)
            cv2.imwrite(self.image_path + image_name, color_image)
            self.image_index += 1

        except CvBridgeError as e:
            rospy.logerr(f"Image conversion failed: {e}")
            return

        cv2.waitKey(1)


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

def main():
    rospy.init_node('stretch_hello_image_capture', anonymous=True)
    rospy.loginfo("begin capturing hello robot image...")

    try:
        sync = ImageSynchronizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()  # Close any OpenCV windows when the node shuts down

if __name__ == '__main__':
    main()
