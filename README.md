# Tutorial ROS 

## Setup

The easy  way to start with  ROS is to  use docker. A set  of official
images are maintained on dockerhub. We are going to use kinematic.

```
    docker pull docker pull osrf/ros:kinetic-desktop-full
```

For     a    brief     overview,     check     out    the     official
base [image](https://hub.docker.com/_/ros/).

We are going to tag this image to ros for convenience.

```
    docker tag osrf/ros:kinetic-desktop-full ros
```

 In addition, we  see that ROS uses the ~/.ros/  directory for storing
 logs, and debugging  info. If you wish to persist  these files beyond
 the  lifecycle of  the containers  which produced  them, the  ~/.ros/
 folder can be mounted to an external volume on the host, or a derived
 image can  specify volumes  to be  managed by  the Docker  engine. By
 default, the container runs as the root user, so /root/.ros/ would be
 the full path to these files.
 
 Create a ros-uto directory and into that dir, to start a container, just use
 
 ```
    docker run -v $PWD/.ros:/root/.ros $PWD/opt:/opt ros
 ```

The problem with that is that we  will not be able to access a display
within the container. 

### mac user

Follow  this post  to pass  the display  to the  container when  using
[docker-for-mac](https://fredrikaverpil.github.io/2016/07/31/docker-for-mac-and-gui-applications/).

One small edge  case I came across, was my  internet connection was on
en1 (WiFi)  rather than en0 (Ethernet),  so the command to  get the ip
needed to be modified as appropriate:

```
    ip=$(ifconfig en1 | grep inet | awk '$1=="inet" {print $2}')
```
At the end, after running xquartz, run the container using

```
    ip=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
    xhost + $ip
    docker run -e DISPLAY=$ip:0 -v /tmp/.X11-unix:/tmp/.X11-unix -v $PWD:/root  ros
```

### linux user

On linux, just use

```
    docker run -it -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1  -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $PWD:/root -v /home/cthorey/.bashrc:/root/.bashrc ros:indigo
```
 
Once in the container, source directly /opt/ros/kinetic/setup.bash, not ros_entrypoint if you dont want your session to exit every so often..

## Ressources 

-  [opencv interface](http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython)
- [dodcker images](https://hub.docker.com/r/osrf/ros/tags/)
- [docker starting point](http://teslacore.blogspot.co.uk/2016/07/docker-for-our-ros-robotic-overlords.html)
- [good-starting-point-tuto](http://moorerobots.com/blog)
- [cheatsheet](https://rych.dcc.uchile.cl/lib/exe/fetch.php?media=documentacion:roscheatsheet_catkin.pdf)
- [movit-tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/)



