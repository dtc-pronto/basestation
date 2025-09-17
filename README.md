# DTC Basestation

## Running the basestation
You can specify what to want tto run in the basestation docker container at runtime with flags, use:
```
--rtk #runs rtk broadcaster
--viz #runs the geovizualizer
--mocha #runs mocha
--sender #runs the scorecard sender node
--all #runs everything
--help
```
you can use `./run.bash` with the flags you want for example: `./run.bash --rtk --mocha` runs the rtk broadcaster and mocha on the basestation.

## Basestation Setup
Clone with `git clone --recursive https://github.com/dtc-pronto/basestation`
then build with `cd basesation && ./build.bash`

This repo should contain the following repos:
- ~~Operator Visualizer~~
- ~~Scoring Server Sender~~
- Task Database / Allocator
- ~~RTK~~
- ~~Supervisor visualier~~
