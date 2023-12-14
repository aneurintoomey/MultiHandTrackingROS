# ROS Multi Hand Tracking Service

## Installation

This project is built on a ROS1 Melodic Docker Image. As a result an installation of the docker engine is a requirement for use. Docker engine installation can be accomplished by installing "Docker Desktop" on Mac/Windows (https://www.docker.com/products/docker-desktop/) or by running `sudo apt install docker` on Ubuntu linux.

Once the Docker Engine is properly installed the application should be built by running `sudo docker build -t dualhandtracking .` in the project's root directory. This will build the docker image allowing the service to be used on the host machine. To access the docker image run `sudo docker run --rm -it --net=host dualhandtracking` this will open a root user shell which can be used to run the service or the demos (note that when I state "within the docker image" I mean in the shell created by docker run).

## Running the Demo

Before running the docker image run `xhost local:docker` to ensure that the dockerfile can see the desktop x server. If you are using Windows/Mac you will have to use an alternative to allow docker to access your display. Within the docker image navigate to the workspaces/multi_hand_tracking_ws directory. Run `source devel/setup.bash` and then `python3.8 src/joint_tracking/scripts/hand_tracking_mediapipe.py &` to start the service (note that the ampersand runs the program in the background meaning you can run another command in the same shell). Once the service has been started run the demo using `rosrun joint_tracking image_demo`. Assuming that the ros master, hand tracking service and iai_kinect node have all been started the program will display an image containing the camera output with the hand landmarks transposed on top.

## Modifying the Service

The entirety of the Hand Tracking service is contained within the "hand_tracking_mediapipe.py" script. This means that any edits made to the service should be made directly to that python script. Note that ROS melodic uses python2.7 but the python script itself needs to be run with python3.8. The python script uses the [ros_numpy](https://github.com/eric-wieser/ros_numpy/tree/master) package rather than the usual cv_bridge ros package due to version compatability issues. The python script sometimes won't run if the devel/setup.bash hasn't been sourced because it won't be able to find the ros_numpy package.

When modifying the service to use a different service call you will have to modify the "MediapipeTracker.srv" file in the "srv" folder. In the case that you use a new package for your message types you will need to add those messages to the CMake file generate_messages() prompt. Finally if you add a new message or service type you will need to add that to the add_message_files or add_service_files prompt in the CMakeLists.txt file.

## Using the service

Once installation has completed any ROS project using the same ROS master can use the hand_tracking service. However, since the MediapipeTracker.srv and Hand.msg files are custom they will have to be integrated into any project attempting to use the service. Besides these details the service is used in a similar fashion to other ROS services. Note that since the docker image uses the `--net=host` to run, any ros master started on the host machine or the docker image will be shared.

## Issues

1. Service is Synchronous: The service can only run one instance at a time. This damages the benefits of multithreading as it could act as a bottleneck if queried too often. If necessary this issue can be offset by converting the service into a ROS node though this may require extra planning to achieve proper pointcloud to image sync.

2. Camera Performance: The Google mediapipe sometimes struggles to identify a hand too far away from the camera, a hand with too much exposure or an image with too much motion blur. Ideally this would be resolved by improving either the camera or the environment within which the hands are tracked.

3. Mediapipe Performance: The Google mediapipe struggles to detect hands that are held perpendicular to the camera view. This could be resolved by using a second camera from the side view and combining the output from each camera to more accurately obtain hand position.

4. Hand Tracking Performance: Sometimes the mediapipe may detect a third hand. If this occurs the mediapipe may not track a left or right hand visible in the frame. This would result in the left/right hand position not being sent through te service.

## Relevant Tutorials

[Docker](https://docs.docker.com/guides/get-started/)
[ROS 1 Melodic](https://wiki.ros.org/ROS/Tutorials)
[ROS w/Docker](https://wiki.ros.org/docker/Tutorials/Docker)

## Author
Aneurin Toomey 
Preferred Email: atoomey@mines.edu