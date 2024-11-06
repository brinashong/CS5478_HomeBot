#!/usr/bin/env python3
import os
from pathlib import Path

import rospy
import cv2
from pathlib import Path
from task_handler.msg import Object, Objects
from geometry_msgs.msg import Point, Quaternion, Pose

import os
from ultralytics import YOLO

# clean up the output directory before each running
def cleanup_image_path(img_path):
    if not os.path.exists(img_path):
        os.makedirs(img_path)
    else:
        for path in Path(img_path).glob("**/*"):
            if path.is_file():
                path.unlink()
                
        rospy.loginfo("Clear up the image folder:" + img_path)



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
        results = yolo_object_detector.predict(rgb_image, save=True, conf=0.3, imgsz = 640)

        boxes = results[0].boxes
        object_labels = results[0].names
        if len(boxes) > 0:
            msg = Objects()
            for i, box in enumerate(boxes.xyxy.tolist()):
                obj = Object()
                x1, y1, x2, y2 = box
                # Save the imags for reviewing: Crop the object using the bounding box coordinates
                ultralytics_crop_object = rgb_image[int(y1):int(y2), int(x1):int(x2)]
                file_name = os.path.join(output_dir, latest_file_short_name.rsplit(".", 1)[0] + "_" + str(i) + ".png")
                cv2.imwrite(file_name, ultralytics_crop_object)
                print(f"Save cropped image to {file_name}")
                
                # capture the target object label, and x centroid, y centroid position
                obj.name = object_labels[int(boxes.cls[i])]
                x_center, y_center = int((x1 + x2) / 2), int((y1 + y2) / 2)

                # capture the distance from x centroid, y_centroid to the camera
                depth_image = cv2.imread(os.path.join(input_depthimage_dir, depth_image_short_name), flags= cv2.IMREAD_ANYDEPTH)
                if depth_image is not None:
                    depth_image = cv2.rotate(depth_image, cv2.ROTATE_90_CLOCKWISE)
                    distance = depth_image[y_center, x_center]

                # store in object
                obj.pose = Pose(Point(x_center, y_center, distance), Quaternion(0.0, 0.0, 0.0, 0.0))
                obj_list.append(obj)
            
            out_filename = os.path.join(output_dir, latest_file_short_name)
            results[0].save(out_filename)
            msg.objects = obj_list
    
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

    try:
        rospy.init_node('stretch_hello_obj_loc_publisher', )
        rospy.loginfo("begin sending object detection info...")
        obj_pub = rospy.Publisher('/objects_location', Objects, queue_size=100)
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            msg = process_image(imageSaveDir, depthImageSaveDir, imageOutputDir)

            if msg is not None:
                obj_pub.publish(msg)

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows() 

    # process_image(imageSaveDir, depthImageSaveDir, imageOutputDir)