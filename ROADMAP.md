# setup

```
      docker run -it --name sgs -p 8080:8080 -p 8888:8888 -p 8181:8181 -p 7681:7681 -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1  -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $PWD/fh_desc:/workspace/fh_desc -v $PWD/smart_grasp_moveit_config:/workspace/smart_grasp_moveit_config -v $PWD/smart_grasping_sandbox:/workspace/smart_grasping_sandbox -v /home/cthorey/.bashrc:/workspace/.bashrc shadowrobot/smart_grasping_sandbox
```
## notes


- if you dont want to use catkin build, do catkin clean, catkin_make

