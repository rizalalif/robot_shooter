
# INSTALLATION ROS2 ON DOCKER 

## Setup ROS2 Workspace

 ### 1. Create container from ros2 images
    docker pull osrf/ros:humble-desktop
    docker run -it --name <name> osrf/ros:humble-desktop -e DISPLAY=host.docker.internal:0.0
    

    
 ### 2. Access ros2
    source /opt/ros/humble/setup.bash

 ### 3. Install Xlaunch
- Display = 0
- Next >> finish

### 4. Create docker volume folder on your pc directory
    mkdir ros2_ws (pc env)
    /root/ros2ws (container env)
   
   build volume from that directory when building the container

###  4. Create a workspace
    mkdir ~/ros2_ws/src
    cd ~/ros2_ws/src
    colcon build

###  5. Add source environment package to .bashrc
    echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
    
 <!-- ### 4. Access rqt_graph (GUI)
    rqt_graph -->

 