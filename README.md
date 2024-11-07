# CS5478_HomeBot

# Running Step </br>
1. Install ultralytics to run Yolo model
command line, type `pip install ultralytics`

2. Run the following commands in different Terminals.
```
roslaunch stretch_navigation navigation_gazebo.launch

roslaunch stretch_config demo_gazebo.launch

rosrun task_handler task_handler_node

rosrun task_handler stretch_image_capture.py

rosrun task_handler stretch_location_publisher.py
```

# Useful Information</br>
- node stretch_image_capture will generate two folders under task_handler folder. </br>
    a. `images` folder - store RGB images </br>
    b. `depth_images` folder - store depth info of the RGB images (with the same index in file name)

    Note: Feel free to delete those images. If node `stretch_image_capture` did not identify the existence of those folders, it will create them automatically.

- node `stretch_location_publisher` will publish the target objects pose info to the topic `objects_location`. </br>
    - It will generate `output_images` folder. It stores tagged images with captured target objects after Yolo prediction. Each running, this folder will get cleaned up.

- folder `perception_model` contains several Yolo models for perception. </br>
    - Yolo_tiny_default </br>
    This folder contains the Yolo3_tiny model which is the default model mentioned by `stretch_ros` tutorial code. However, our experiments show it is not advantageous compared to higher version Yolo models.

    - Yolo8_retrained </br>
    This folder contains Yolo8 retrained model by using around 400 images from the Small House environment.

    - Yolo11_retrained </br>
    This folder contains Yolo11 retrained model. Same retraining process as Yolo8 retrained model. Compared to Yolo8 retrained model, its performance and precision both improve. We are using this model to identify objects.