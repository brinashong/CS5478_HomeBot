#!/usr/bin/env python3
import os
from pathlib import Path

import rospy
import cv2
from pathlib import Path
from task_handler.msg import Object, Objects
from geometry_msgs.msg import Point, Quaternion, Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
import tf
import numpy as np
import os
from ultralytics import YOLO
from sensor_msgs.msg import CameraInfo
from gazebo_msgs.srv import GetModelState

bridge = CvBridge()

# clean up the output directory before each running
def cleanup_image_path(img_path):
    if not os.path.exists(img_path):
        os.makedirs(img_path)
    else:
        for path in Path(img_path).glob("**/*"):
            if path.is_file():
                path.unlink()
                
        rospy.loginfo("Clear up the image folder:" + img_path)



def pixel_to_world(u, v, depth, camera_info, camera_frame, world_frame):
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
        tf_listener.waitForTransform(world_frame, camera_frame, rospy.Time(0), rospy.Duration(4.0))
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


def process_image(input_rgbimage_dir, input_depthimage_dir, output_dir):
    msg = None
    obj_list = []

    # stretch_image_capture.py usually stores 20 latest images in the folder. When anayzing
    # images and looking for the target objects, choose only the latest image
    filenames = [os.path.join(input_rgbimage_dir, fname) for fname in os.listdir(input_rgbimage_dir)]
    latest_file = max(filenames, key=os.path.getmtime)
    latest_file_short_name = latest_file.rsplit("/", 1)[1]

    file_parts = latest_file_short_name.split("_")
    depth_image_short_name = file_parts[0] + "_depth_" + file_parts[1]
    
    rgb_image = cv2.imread(latest_file)

    if rgb_image is not None:
        # the original image comes out sideways. Rotate it to upright
        rgb_image = cv2.rotate(rgb_image, cv2.ROTATE_90_CLOCKWISE)
        results = yolo_object_detector.predict(rgb_image, save=True, conf=0.6, imgsz = 640)

        boxes = results[0].boxes
        if len(boxes) > 0:
            msg = Objects()
            for i, box in enumerate(boxes.xyxy.tolist()):
                obj = Object()
                x1, y1, x2, y2 = box
                # Currently not save single identified objects to imporove performance. If you want to see each identified object, uncomment the following block
                # Save the imags for reviewing: Crop the object using the bounding box coordinates
                
                # ultralytics_crop_object = rgb_image[int(y1):int(y2), int(x1):int(x2)]
                # file_name = os.path.join(output_dir, latest_file_short_name.rsplit(".", 1)[0] + "_" + str(i) + ".png")
                # cv2.imwrite(file_name, ultralytics_crop_object)
                # print(f"Save cropped image to {file_name}")
                
                # capture the target object label, and x centroid, y centroid position
                class_id = int(boxes.cls[i])
                obj.id = object_class_label[class_id][0]
                obj.name = object_class_label[class_id][1]
                x_center, y_center = int((x1 + x2) / 2), int((y1 + y2) / 2)

                if class_id in [2, 3, 4]: 
                    # capture the distance from x centroid, y_centroid to the camera
                    depth_image = cv2.imread(os.path.join(input_depthimage_dir, depth_image_short_name), flags= cv2.IMREAD_ANYDEPTH)
                    if depth_image is not None:
                        depth_image = cv2.rotate(depth_image, cv2.ROTATE_90_CLOCKWISE)
                        distance = depth_image[y_center, x_center]

                    # convert the location in camera to world coordinate
                    world_coords = pixel_to_world(x_center, y_center, distance, camera_info, camera_frame, world_frame)
                    if world_coords is not None:
                        rospy.loginfo("World coordinates: x=%f, y=%f, z=%f", *world_coords)
                        # store in object
                        obj.pose = Pose(Point(*world_coords), Quaternion(0.0, 0.0, 0.0, 0.0))
                        obj_list.append(obj)

            out_filename = os.path.join(output_dir, latest_file_short_name)
            results[0].save(out_filename)
            msg.objects = obj_list
    
            tagged_image = cv2.imread(out_filename)
            cv2.imshow("Tagged Image",tagged_image)
            cv2.waitKey(1)
    
            # if you want to show the image in Rviz, uncomment the following line, and add ros_image to return value
            # ros_image = bridge.cv2_to_imgmsg(tagged_image,encoding="bgr8")

    # temporarily for debugging purpose
    print(msg)
    return msg


if __name__ == '__main__':

    models_directory = os.path.dirname(os.path.abspath(__file__)) + "/perception_model/Yolo11_retrained/"
    yolo_object_detector = YOLO(models_directory + "best.pt")
    rospy.loginfo('The directory for deep learning models:', models_directory)  

    imageSaveDir = os.path.dirname(os.path.abspath(__file__)) + "/images"
    depthImageSaveDir = os.path.dirname(os.path.abspath(__file__)) + "/depth_images"
    imageOutputDir = os.path.dirname(os.path.abspath(__file__)) + "/output_images"

    cleanup_image_path(imageOutputDir)

    # the recognized objects. Value represents (id, name) in Object.msg
    object_class_label = {0: ("table", "table"), 
                   1: ("rubbish_bin","rubbish_bin"),
                   2: ("book", "Book"),
                   3: ("can", "Coke"),
                   4: ("cup", "Mug"),
                   5: ("book_shelf", "book_shelf")}
    # object_class_label = {
    #                        0: ("book", "Book"),
    #                        1: ("can", "Coke"),
    #                        2: ("cup", "Mug")
    #                      }

    try:
        rospy.init_node('stretch_hello_obj_loc_publisher', anonymous=True)
        rospy.loginfo("begin sending object detection info...")
        obj_pub = rospy.Publisher('/objects_poses', Objects, queue_size=100)

        # if you want to show image in rviz, uncomment the following line to publish to the topic /sensor_msgs/Image
        # rviz_camera = rospy.Publisher("/sensor_msgs/Image", Image, queue_size=100)

        rate = rospy.Rate(1)

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
                if object_name == "Coke":
                    gt_object.pose.position.z += 0.07
                elif object_name == "Mug":
                    gt_object.pose.position.z += 0.08
                elif object_name == "Book":
                    gt_object.pose.position.z += 0.13
                gt_objects_info.append(gt_object)

            if response.success:
                rospy.loginfo("Model_states received.")
        except rospy.ROSException as e:
            rospy.logerr("Fail to retrieve model states.")

        print("object ground truth:")
        rospy.loginfo(gt_objects_info)
        print("")

        while not rospy.is_shutdown():
            msg = process_image(imageSaveDir, depthImageSaveDir, imageOutputDir)

            if msg is not None:
                obj_pub.publish(msg)
                # rviz_camera.publish(rviz_img)

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows() 
