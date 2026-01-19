# DTC Basestation ROS2 Port

## Running the basestation

You can build using the build-basestation.bash file and run using the run-basestation.bash file. The image does not execute the launch file automatically

so you can run it with the command

> ros2 launch scorecard_submitter scorecard.launch.py

## Scope of the Port

The port only ported over the scoring server submission, tested with the DARPA test server (not with spoofer yet)
