# CS5478_HomeBot

# Running Step </br>
1. Install ultralytics to run Yolo model
command line, type `pip install ultralytics`

2. Run the following commands in different Terminals.
    ```
    roslaunch stretch_navigation navigation_gazebo_robotiq.launch

    roslaunch task_handler tf.launch

    roslaunch stretch_robotiq_moveit_config demo_gazebo.launch

    rosrun task_handler task_handler_node

    rosrun task_handler stretch_image_capture.py

    rosrun task_handler stretch_location_publisher.py
    ```

    - If you just want to call /get_objects service when needed, replace the last two rosrun commands with:

    ```
    rosrun task_handler stretch_image_capture.py

    Replace the above with: rosrun task_handler stretch_identify_object_server.py

    rosrun task_handler stretch_location_publisher.py

    Replace the above with: rosrun task_handler stretch_identify_object_client.py
    ```

# Useful Information</br>
There are two methods to get object information through camera. <br>

1. Subscription:
    - node stretch_image_capture will generate two folders under task_handler folder. </br>
        a. `images` folder - store RGB images </br>
        b. `depth_images` folder - store depth info of the RGB images (with the same index in file name). <br>
        c. `output_images` folder - store the tagged images by the Yolo11 model. <br>

        Note: Feel free to delete those images. If node `stretch_image_capture` did not identify the existence of those folders, it will create them automatically.

    - node `stretch_location_publisher` will publish the target objects pose info to the topic `objects_poses`. </br>
        - It will generate `output_images` folder. It stores tagged images with captured target objects after Yolo prediction. Each running, this folder will get cleaned up.
    <br>

2. Service:
    - node `stretch_identify_object_server.py` will start the service at `/get_objects`.
    - node `stretch_identify_object_client.py` contains sample code on how to request the service.<br>

3. Folder `perception_model` contains two Yolo models for perception. </br>
    - Yolo8_retrained </br>
    This folder contains Yolo8 retrained model by using around 400 images from the Small House environment.

    - Yolo11_retrained </br>
    This folder contains Yolo11 retrained model. Same retraining process as Yolo8 retrained model. Compared to Yolo8 retrained model, its performance and precision both improve. We are using this model to identify objects.

      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0" 
