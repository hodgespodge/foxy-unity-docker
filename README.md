# Unity and ROS2 Foxy with Nav2 SLAM Example

This project is an amalgamation of [game-ci](https://github.com/game-ci/docker) and [Robotics-Nav2-SLAM-Example](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example)

- Ubuntu version: 20.04
- unity3D version: 2020.3.11f 
- ROS2 version: foxy

**Setup**

- If you have a .ulf license file already, copy it to the shared folder. Otherwise license instructions will be provided later.
- The shared volume is shared between the container and the local host. The shared readme has a few commands you can run in the container.

build the docker with: 

> docker build -t ros-unity:foxy .

**How to run**

- I run docker using rocker for graphics card support. You may have an alternate way of running the container with graphics. 
- It's important that the shared volume is included since that's where Unity looks for the license.

> rocker --nvidia --x11 --volume $(pwd)/shared:/shared --   ros-unity:foxy 

Rocker note: you only need to start/run the container with rocker. You can exec in as normal.

**How to exec in (connect from another window)**

In a new window, get container name with 

> docker ps

Then connect with

> docker exec -it \<container name\> /bin/bash


**TODO**

- Make container user non-root