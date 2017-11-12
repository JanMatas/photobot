# photobot
Project for 4th year course Human Centered Robotics at Imperial College London

## Git workflow

We have 2 main branches: 
- *master* - this should always contain stable tested code. We will use this for demos.
- *develop* (default branch) - this is the branch we develop against. It should always contain latest, reviewed code that might not be integration tested yet.

During our usual workflow, those branches are protected and we should never commit code there directly. Instead, we always create new branch named after the feature or bugfix it contains and it should be based on develop (example branch names: "camera-integration", "speech-synthesis-skeleton", "bugfix/display-overflow"). 

After you are done, you should create a pull request to develop branch and ask someone else to review it. When the code is approved (and both the author and reviewer are convinced it works), we merge to develop. Every week (or as required), we take code on *develop*, test it properly and merge to *master*.

## Dependencies:

Some dependencies that need to be manually installed:

- *pip packages* - gtts, playsound, apiai
- *apt get* - ffmpeg
### Camera
- *apt* - ros-kinetic-image-common 
### RosAria
- Get the package from http://robots.mobilerobots.com/ARIA/download/current/libaria_2.9.1a+ubuntu16_amd64.deb
- Install it with dpkg -i libaria_2.9.1a+ubuntu16_amd64.deb
- Go to /usr/local/Aria and run `make clean; make`
### Navigation stack
```
ros-kinetic-move-base
ros-kinetic-depthimage-to-laserscan
ros-kinetic-slam-gmapping
ros-kinetic-map-server
```

## Running RosAria
`sudo chmod a+rwx /dev/ttyUSB0`

`sudo chmod a+rwx /dev/ttyUSB1`

`rosrun rosaria RosAria _port:=/dev/ttyUSB0` change to different USB if it doesn't work

## Running joystick
`sudo chmod a+rwx /dev/input/js0`

`rosrun joy joy_node _dev:=/dev/input/js0`

`rosrun photobot_joy teleop.py`
