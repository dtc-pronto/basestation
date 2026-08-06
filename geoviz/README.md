# DTC Basestation Visualizer

This tool always you to interface position topics with a MapBox satellite map. It is a ros package called `geoviz`. It subscirbes to the robot locations and casualties and plots them on an interactive satillite map.

### Running

Run this with `roslaunch geoviz viz.launch`, this will start the map server on your local machine. To view the map enter `127.0.0.1:5000` into your browser.

You can also run this on a remote server by setting the ip and port

```
roslaunch geoviz viz.launch ip:=<ADDRESS> port:=<PORT>
```

You can configure what robots are in the swarm in `config/map.yaml` under the `robots` parameter.

### Key Setup

You'll need to setup a mapbox public api key, directions are [here](https://docs.mapbox.com/help/dive-deeper/access-tokens/) and add the key to `env/mapbox_token.env`.

### Authors

- Jason Hughes - `jasonah.at.seas.upenn.edu`


# GeoViz

GeoViz is a ROS-based web application for visualizing robot and casualty locations on an interactive Mapbox satellite map. It subscribes to robot position topics, renders the live locations of all robots, and displays casualty markers through a browser-based interface.

The visualization can be hosted locally or on a remote machine, making it useful for monitoring from the basestation.

> GeoViz requires a Mapbox access token to display satellite imagery.

## Repo layout

```text
.
├── config/                 # Map configuration and robot list
├── env/                    # Environment files (Mapbox access token)
├── launch/                 # ROS launch files
├── scripts/                # Main ROS node and utility scripts
├── src/                    # Python package source
├── static/                 # JavaScript and static web assets
├── templates/              # HTML templates
├── tiles/                  # Offline map tiles
├── CMakeLists.txt
├── package.xml
├── setup.py
└── README.md
```

## Configuration

Before running GeoViz, configure the following files.

### Mapbox Token

Create a Mapbox public access token following the instructions below.

https://docs.mapbox.com/help/dive-deeper/access-tokens/

Add the token to:

```text
env/mapbox_token.env
```

### Robot Configuration

Edit

```text
config/map.yaml
```

to configure:

- Robots in the swarm
- Default map center
- UTM zone
- Magnetic declination
- Map offset

## Run

Launch GeoViz locally:

```bash
roslaunch geoviz viz.launch
```

By default the web server runs on

```
http://127.0.0.1:5051
```

To host the server on another machine:

```bash
roslaunch geoviz viz.launch ip:=<ADDRESS> port:=<PORT>
```

## Launch Arguments

| Argument  | Default       | Description                     |
| --------- | ------------- | ------------------------------- |
| `ip`    | `127.0.0.1` | Address to host the web server  |
| `port`  | `5051`      | HTTP server port                |
| `gps`   | `true`      | Display GPS data                |
| `odom`  | `true`      | Display odometry data           |
| `glins` | `true`      | Display GLINS localization data |

## ROS Interfaces

### Subscribed Topics

The launch file remaps the following topics by default:

| Topic          | Description        |
| -------------- | ------------------ |
| `/ublox/fix` | Robot GPS position |
| `/Odometry`  | Robot odometry     |

Additional robot and casualty topics are configured through `config/map.yaml`.

## Notes

- Includes offline map tiles under `tiles/` for supported regions.
