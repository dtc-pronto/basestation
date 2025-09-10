# How to use FPV Viewer on base station or other machines

1. build the docker image with the build.bash script
2. make sure you're on the same network as the Jackals
3. run the desired bash script (run.bash or run-many.bash)

**for a singular instance,** 
* use the run.bash script to launch the GUI
* select which Jackal you would like to connect to
* input a UDP port and the IP of the device that is running the GUI
* input the resolution, fps, and mode parameters
* start streaming with the `Start View + Apply` button

**for multiple instances**
* use run-many.bash script followed by the number of instances to launch (ex. `./run-many.bash 3` for 3 instances)
* Follow the steps for a **singular instance**, making sure to note that the UDP ports must be different for each instance