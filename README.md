# photobot
Project for 4th year course Human Centered Robotics at Imperial College London

## Git workflow

We have 2 main branches: 
- *master* - this should always contain stable tested code. We will use this for demos.
- *develop* (default branch) - this is the branch we develop against. It should always contain latest, reviewed code that might not be integration tested yet.

During our usual workflow, those branches are protected and we should never commit code there directly. Instead, we always create new branch named after the feature or bugfix it contains and it should be based on develop (example branch names: "camera-integration", "speech-synthesis-skeleton", "bugfix/display-overflow"). 

After you are done, you should create a pull request to develop branch and ask someone else to review it. When the code is approved (and both the author and reviewer are convinced it works), we merge to develop. Every week (or as required), we take code on *develop*, test it properly and merge to *master*.

## Dependencies:
We are using `ros-kinetic`

- *pip packages* - gtts, playsound, apiai
- *apt get* - ffmpeg
### Camera
- ros-kinetic-image-common 
### RosAria
- Get the package from http://robots.mobilerobots.com/ARIA/download/current/libaria_2.9.1a+ubuntu16_amd64.deb
- Install it with dpkg -i libaria_2.9.1a+ubuntu16_amd64.deb
- Go to /usr/local/Aria and run `make clean; make`
- Run `sudo apt install ros-kinetic-rosaria`

### Navigation stack
```
ros-kinetic-move-base
ros-kinetic-gmapping
ros-kinetic-map-server
ros-kinetic-rplidar
```

## Running the robot
You need to obtain Google API credentials to work with the speech API.
Place them somewhere and point the `launch/credentials.txt` file to them.

`source launch/credentials.txt` to export API keys
`roslaunch kinect_launch.launch` to track faces and do centering when
taking a picture
`roslaunch face_detection_v1.launch` to run the speech services, website 
and the emailer nodes

## Running the navigation
We have created a `udev` rule to remap the names of lidar and the robot. We
use these names in the `chmod-it.sh` file.
To run the navigation stack (independent of other nodes) do:
```
cd launch
sudo sh chmod-it.sh
roslaunch nav.launch
```
With navigation stack up, run `rviz` and load the `awesome_conf.rviz`. This
will show the map you have produced with `gmapping` and the initial robot
position estimate. Now you need to do
`rosrun photobot_navigation_goals record_goals.py` and the click on the points
you want the robot to follow in subsequent runs.
To make the robot start moving do:
`rosrun photobot_navigation_goals move_around.py` which will follow the
previously recorded points.

## Project structure
### Navigation
We have a `move_base` folder that includes configuration for the base ROS
navigation stack, including robot dimensions and topic and coordinate frame
names.

We have `photobot_laser_filter` node that restricts the angular output of 
the laser scanner so that it doesn't include the parts of the robot in the scan.

There is `photobot_navigation_goals` package that has a node to record the 
route for the robot and a node to follow that route and stop if there is 
interaction with a person.