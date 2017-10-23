# photobot
Project for 4th year course Human Centered Robotics at Imperial College London

## Git workflow

We have 2 main branches: 
- *master* - this should always contain stable tested code. We will use this for demos.
- *develop* (default branch) - this is the branch we develop against. It should always contain latest, reviewed code that might not be integration tested yet.

During our usual workflow, those branches are protected and we should never commit code there directly. Instead, we always create new branch named after the feature or bugfix it contains and it should be based on develop (example branch names: "camera-integration", "speech-synthesis-skeleton", "bugfix/display-overflow"). 

After you are done, you should create a pull request to develop branch and ask someone else to review it. When the code is approved (and both the author and reviewer are convinced it works), we merge to develop. Every week (or as required), we take code on *develop*, test it properly and merge to *master*.
