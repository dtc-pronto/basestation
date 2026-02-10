# DTC Basestation ROS2 Port

## Running the basestation

You can build using the build-basestation.bash file and run using the run-basestation.bash file. The image does not execute the launch file automatically so you can run it with the command. run-basestation.bash will have to be modified to add rosbags folder to run. The scoring server also need to be running for the launch file to execute without error.

> ros2 launch scorecard_submitter scorecard.launch.py

## Scope of the Port

The port only ported over the scoring server submission, tested with the DARPA test server.
