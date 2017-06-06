# Setup

```
docker run -it --name sgs -p 8080:8080 -p 8888:8888 -p 8181:8181 -p 7681:7681 -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1  -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $PWD/fh_desc:/workspace/src/fh_desc -v $PWD/smart_grasp_moveit_config:/workspace/src/smart_grasp_moveit_config -v $PWD/vision_sytem:/workspace/src/vision_system -v $PWD/smart_grasping_sandbox:/workspace/src/smart_grasping_sandbox -v /home/cthorey/.bashrc:/workspace/.bashrc ros:sandbox bash
```

## Usefull link

- [rgbdslam setup](https://github.com/felixendres/rgbdslam_v2). I did build g2o but when trying to get catkin build, I got an error with qt.

