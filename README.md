# CS5478_HomeBot

# Running Step </b>
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

3. Useful Information</b>
- node stretch_image_capture will generate two folders under task_handler folder. 
    a. `images` folder - store RGB images
    b. `depth_images` folder - store depth info of the RGB images (with the same index in file name)

    Note: Feel free to delete those images. If node `stretch_image_capture` did not identify the existence of those folders, it will create them automatically.

- node `stretch_location_publisher` will publish the target objects pose info to the topic `objects_location`. </b>
    - It will generate `output_images` folder. It stores tagged images with captured target objects after Yolo prediction. Each running, this folder will get cleaned up.


