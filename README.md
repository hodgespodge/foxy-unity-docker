# Unity and ROS2 Foxy with Nav2 SLAM Example

This project is an amalgamation of [game-ci](https://github.com/game-ci/docker) and [Robotics-Nav2-SLAM-Example](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example)

The unity3D version is 2020.3.11f 

**
How I build
**
docker build -t foxy-unity:hub .


**How I run **

rocker --nvidia --x11 --volume $(pwd)/shared:/shared --   foxy-unity:hub 

# TODO 

Right now, the idea is to put one's Unity_lic.ulf file into the shared folder so that you don't have to make a new each time you launch the container. 

The problem is, right now I don't know how to make unity-hub recognize the Unity_lic.ulf file in the build process. 