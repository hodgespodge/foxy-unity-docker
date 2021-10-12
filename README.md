# Unity and ROS2 Foxy with Nav2 SLAM Example

This project is an amalgamation of [gui-docker](https://github.com/bandi13/gui-docker), [game-ci](https://github.com/game-ci/docker) and [Robotics-Nav2-SLAM-Example](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example)

- Ubuntu version: 20.04
- unity3D version: 2020.3.11f 
- ROS2 version: foxy

**Setup**

- If you have a .ulf license file already, copy it to the shared folder. Otherwise license instructions will be provided upon running the container.
- The shared volume is shared between the container and the local host. The shared readme has a few commands you can run in the container.

build the docker with: 

> docker build -t ros-unity:foxy .

**How to run**

> docker run --shm-size=256m -it -p 5901:5901 -e VNC_PASSWD=123456  -v $(pwd)/shared:/shared foxy-unity:vnc

Your VNC_PASSWD should match whichever password you define in the dockerfile

**How to use once running**

Go to the noVNC HTML client url in your favorite web browser and connect.

You can right click and go to Applications>Shells>Bash to get a working terminal. 

For convenience, the following aliases have been provided

> alias rviz_example = ros2 launch unity_slam_example unity_slam_example.py
>
> alias unity_example = /opt/unity/editors/2020.3.11f1/Editor/Unity -projectPath /Robotics-Nav2-SLAM-Example/Nav2SLAMExampleProject/

**Note**

If your license is included but unity still complains, run 
> /opt/unity/editors/2020.3.11f1/Editor/Unity -batchmode -manualLicenseFile /shared/*.ulf -logfile