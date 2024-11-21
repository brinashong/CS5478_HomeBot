# CS5478_HomeBot

# Running Step </br>
1. Install ultralytics to run Yolo model
command line, type `pip install ultralytics`

2. Run the following commands in different Terminals.
```
roslaunch stretch_navigation navigation_gazebo.launch

roslaunch stretch_moveit_config demo_gazebo.launch

rosrun task_handler task_handler_node

rosrun task_handler stretch_image_capture.py

rosrun task_handler stretch_location_publisher.py
```

# Useful Information</br>
- node stretch_image_capture will generate two folders under task_handler folder. </br>
    a. `images` folder - store RGB images </br>
    b. `depth_images` folder - store depth info of the RGB images (with the same index in file name). <br>
    c. `output_images` folder - store the tagged images by the Yolo11 model. <br>

    Note: Feel free to delete those images. If node `stretch_image_capture` did not identify the existence of those folders, it will create them automatically.

- node `stretch_location_publisher` will publish the target objects pose info to the topic `objects_poses`. </br>
    - It will generate `output_images` folder. It stores tagged images with captured target objects after Yolo prediction. Each running, this folder will get cleaned up.

- folder `perception_model` contains two Yolo models for perception. </br>
    - Yolo8_retrained </br>
    This folder contains Yolo8 retrained model by using around 400 images from the Small House environment.

    - Yolo11_retrained </br>
    This folder contains Yolo11 retrained model. Same retraining process as Yolo8 retrained model. Compared to Yolo8 retrained model, its performance and precision both improve. We are using this model to identify objects.

# Publish object ground truth poses
```
rostopic pub /object_poses task_handler/Objects "objects:
- name: 'can'
  pose:
    position:
      x: 2.85
      y: -1.51
      z: 0.37
    orientation:
      x: 0.0
      y: 0.0
      z: 1.0
      w: 0.0" 
- name: 'can'
  pose:
    position:
      x: 2.08
      y: -1.63
      z: 0.37
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0" 
- name: 'cup'
  pose:
    position:
      x: 2.07
      y: -1.89
      z: 0.37
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0" 
- name: 'book'
  pose:
    position:
      x: 5.74
      y: 0.92
      z: 0.80
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0" 
- name: 'book'
  pose:
    position:
      x: 5.70
      y: 1.03
      z: 0.80
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0" 
```