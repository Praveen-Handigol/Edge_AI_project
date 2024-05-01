# EDGE_AI
Requirements:  ROS2 humble, Ubuntu 22

dataset available at: https://drive.google.com/drive/folders/1uzs-1oOQtPqHe5Yvc_Oq-kZggR0d2W4h?usp=sharing

clone the repo in the src folder of a ROS2 workspace(~/my_ros2_ws/src/) and edit the model_path in EDGE_AI/stackbot_core/stackbot_core/model_publish.py

move the contents of the EDGE_AI folder to the parent folder, your folders should look like
~/my_ros2_ws/src/stackbot_core

change directory to the workspace 

    cd ~/my_ros2_ws

build the working space using

    colcon build 
    source install/setup.bash 

to run the robot with the selected model 

    ros2 launch stackbot_core model_run.launch.py

link for ROS2 doccumentation: https://docs.ros.org/en/humble/index.html

link for making a workspace in ROS2: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html
