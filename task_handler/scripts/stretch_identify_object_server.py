#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
from pathlib import Path
import numpy as np
from task_handler.srv import GetObjects, GetObjectsResponse
import message_filters
from task_handler.msg import Object, Objects
from geometry_msgs.msg import Point, Quaternion, Pose, PointStamped
import tf
from ultralytics import YOLO
from sensor_msgs.msg import CameraInfo, Image
from gazebo_msgs.srv import GetModelState



class GetObjectsServer:
    def __init__(self, yolo_model, camera_info, object_label):
        self.latest_color_image = None
        self.latest_depth_image = None      # cache the latest data
        self.cv_bridge = CvBridge()
        self.yolo_object_detector = yolo_model
        self.camera_info = camera_info
        self.object_label = object_label
        self.output_img_index = 0

        # Subscribers for color and depth images
        self.color_sub = message_filters.Subscriber("camera/color/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("camera/depth/image_raw", Image)
        # Time synchronizer with a 0.1-second tolerance window
        self.sync = message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=1000, slop=0.1)
        self.sync.registerCallback(self.data_callback)

        self.service = rospy.Service("/get_objects", GetObjects, self.handle_request)
        rospy.loginfo("GetObjects server is ready.")


    def data_callback(self, color_msg, depth_msg):
        # Update the cached message (image) from the topic
        try:
            # Convert messages to OpenCV images
            self.latest_color_image = self.cv_bridge.imgmsg_to_cv2(color_msg, "bgr8")
            self.latest_depth_image = self.cv_bridge.imgmsg_to_cv2(depth_msg, "32FC1")  # Assuming depth is in meters
        except:
            rospy.logerr("fail to convert image format to cv2 and cache them.")


    def pixel_to_world(self, u, v, depth, camera_info, camera_frame, world_frame):
        """
        Converts pixel coordinates to world coordinates.
        
        Args:
            u, v: Pixel coordinates in the image.
            depth: Depth value at the pixel (in meters).
            camera_info: Camera intrinsic parameters (from CameraInfo message).
            camera_frame: Frame ID of the camera.
            world_frame: Frame ID of the world.
            
        Returns:
            (x_world, y_world, z_world): World coordinates of the object.
        """
        # Extract intrinsic camera parameters
        fx = camera_info.K[0]  # Focal length in x
        fy = camera_info.K[4]  # Focal length in y
        cx = camera_info.K[2]  # Optical center in x
        cy = camera_info.K[5]  # Optical center in y
        
        # Step 1: Convert pixel to camera coordinates
        X_camera = (u - cx) * depth / fx
        Y_camera = (v - cy) * depth / fy
        Z_camera = depth  # Depth from the depth camera
        
        # Point in the camera frame
        point_camera = np.array([X_camera, Y_camera, Z_camera, 1])
        
        # Step 2: Transform camera coordinates to world coordinates
        try:
            tf_listener = tf.TransformListener()
            tf_listener.waitForTransform(world_frame, camera_frame, rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = tf_listener.lookupTransform(world_frame, camera_frame, rospy.Time(0))
            
            # Create transformation matrix
            trans_mat = tf.transformations.translation_matrix(trans)
            rot_mat = tf.transformations.quaternion_matrix(rot)
            T_camera_to_world = np.dot(trans_mat, rot_mat)
            
            # Transform point to world coordinates
            point_world = np.dot(T_camera_to_world, point_camera)
            return point_world[:3]  # Return (x, y, z) in world coordinates
        except:
            rospy.logerr("TF lookup failed")
            return None


    def handle_request(self, req):
        if self.latest_color_image is None:
            rospy.logwarn("No data received from /camera/color/image_raw yet!")
            return None

        
        # the original image comes out sideways. Rotate it to upright
        rgb_image = cv2.rotate(self.latest_color_image, cv2.ROTATE_90_CLOCKWISE)
        results = self.yolo_object_detector.predict(rgb_image, save=True, conf=0.65, imgsz = 640)
  
        boxes = results[0].boxes
        obj_list = []
        msg = Objects()

        if len(boxes) > 0:
            for i, box in enumerate(boxes.xyxy.tolist()):
                obj = Object()
                x1, y1, x2, y2 = [int(item) for item in box]

                # capture the target object label, and x centroid, y centroid position
                class_id = int(boxes.cls[i])
                obj.name = self.object_label[class_id][1]
                obj.id = self.object_label[class_id][0]
                # conf = boxes[i].conf
                # rgb_image = cv2.rectangle(rgb_image, (x1, y1-25), (x2, y2), (0, 255, 0), 3)
                # rgb_image = cv2.putText(rgb_image, obj.name + str(float(conf)), (x1, y1 - 10), cv2.FONT_HERSHEY_PLAIN, 0.7, (0,0,0), 3)

                x_center, y_center = int((x1 + x2) / 2), int((y1 + y2) / 2)

                # if the class_id indicates Cup, Can or Book, store them in object_list
                if class_id in [2, 3, 4]: 
                    depth_image = cv2.rotate(self.latest_depth_image, cv2.ROTATE_90_CLOCKWISE)
                    depth = depth_image[y_center, x_center]
                    print("debugging...")

                    # convert the location in camera to world coordinate
                    world_coords = self.pixel_to_world(x_center, y_center, depth, self.camera_info, camera_frame, world_frame)
                    if world_coords is not None:
                        rospy.loginfo("World coordinates: x=%f, y=%f, z=%f", *world_coords)
                        # store in object
                        obj.pose = Pose(Point(*world_coords), Quaternion(0.0, 0.0, 0.0, 0.0))
                        obj_list.append(obj)

            rospy.loginfo("finish looping the boxes")
        # show the tagged image
        out_filename = os.path.join(imageOutputDir, "tagged_image_" + str(self.output_img_index) + ".jpg")
        self.output_img_index +=1
        results[0].save(out_filename)

            # if you want to show the image in Rviz, uncomment the following line, and add ros_image to return value
            # ros_image = bridge.cv2_to_imgmsg(tagged_image,encoding="bgr8")

        # temporarily for debugging purpose
        print(msg)
        rospy.loginfo("Finish processing, and response to client.")

        msg.objects = obj_list
        return msg, out_filename

# clean up the output directory before each running
def cleanup_image_path(img_path):
    if not os.path.exists(img_path):
        os.makedirs(img_path)
    else:
        for path in Path(img_path).glob("**/*"):
            if path.is_file():
                path.unlink()
                
        rospy.loginfo("Clear up the image folder:" + img_path)

if __name__ == "__main__":
    rospy.init_node("stretch_identify_object_server")
    imageOutputDir = os.path.dirname(os.path.abspath(__file__)) + "/output_images"
    cleanup_image_path(imageOutputDir)

    # get Yolo model
    models_directory = os.path.dirname(os.path.abspath(__file__)) + "/perception_model/Yolo11_retrained/"
    rospy.loginfo("Loading Yolo11 model from " + models_directory)
    yolo_object_detector = YOLO(models_directory + "best.pt")
    rospy.loginfo('Deep learning model has been loaded.') 

    # get camera info
    try:
        rospy.loginfo("waiting for the camera info...")
        camera_info = rospy.wait_for_message("/camera/color/camera_info", CameraInfo, timeout=5.0)
        rospy.loginfo("CameraInfo received.")
        camera_frame = "camera_link"
        world_frame = "map"
    except rospy.ROSException as e:
        rospy.logerr("Fail to retrieve camera info.")

    gt_objects_info = []
    try: 
        for object_id, object_name in [("can", "Coke"), ("cup", 'Mug'), ("book", 'Book')]:
            rospy.loginfo("Getting the positions of objects in Gazebo.")
            get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            response = get_model_state(model_name = object_name, relative_entity_name="world")
            gt_object = Object()
            gt_object.id = object_id 
            gt_object.name = object_name
            gt_object.pose = response.pose
            gt_objects_info.append(gt_object)

        if response.success:
            rospy.loginfo("Model_states received.")
    except rospy.ROSException as e:
        rospy.logerr("Fail to retrieve model states.")

    print("object ground truth:")
    rospy.loginfo(gt_objects_info)
    print("")

    # the recognized objects. Value represents (id, name) in Object.msg
    object_class_label = {0: ("table", "table"), 
                   1: ("rubbish_bin","rubbish_bin"),
                   2: ("book", "Book"),
                   3: ("can", "Coke"),
                   4: ("cup", "Mug"),
                   5: ("book_shelf", "book_shelf")}

    server = GetObjectsServer(yolo_object_detector, camera_info, object_class_label)
    rospy.spin()

	
