# DTC Basestation

This repo contains all code that runs on the basestation for the DARPA Triage Challenge. The following packages use ROS2 (Jazzy) and run inside the dedicated basetation container:

- **MOCHA (mesh networking)** — Found in the common repo, MOCHA handles comms between basestation and all other robots.
- **rtk-corrections** — Found in the common repo, rtk-corrections broadcasts RTCM messages to all our robots to give thier GPS cm level accuracy
- **geoviz** — A Mapbox-based live map of robots and casualties
- **scoring-server-submission** — Recieves casualty/triage/vitals reports from the robots and forwards them to the DARPA scoring server, gate by gate.
- **dtc-msgs** — Shared message definitions used across the DTC stack

> For more information about any of these packages please refer to thier individual READMEs

## Repo layout

```
.
├── common/                      # Common repo that contains MOCHA/RTK
├── dtc-msgs/                    # dtc-msgs package
├── geoviz                       # geoviz package
├── scoring-server-submission/   # scoring-server-submission package
├── scripts/                     # Contians bash script to auto-detect hardware and launch RTK/MOCHA
├── system-setup/                # Contains udev rules, systemd service, and setup script for the physical basestation
├── Dockerfile                   # Instructions for building the basestation image (ROS2 Jazzy)
├── README.md                    # THATS ME :)
├── build.bash                   # Builds the image
├── entrypoint.bash              # Container entrypoint — decides what to launch
└── run-basestation.bash         # Runs the container
```

## Setup

To clone this repo run the following command so all submodules also get cloned.

```bash
git clone --recurse-submodules <this-repo>
```

Run the following to update any submodules to the most recent commit on thier tracked branch.

```bash
git submodule update --remote --merge <path/to/submodule>
```

If you cloned without --recurse-submodules run

```Shell
git submodule update --init --recursive
```

When this repo is installed on a new machine run the following command in (`system-setup/`) to setup udev rules, network setting, and system services

```Shell
bash setup-basestation.sh
```

## Build

```bash
./build.bash
```

Builds a Docker image (`dtc-platform-<hostname>:basestation`) from `dtcpronto/ros-jazzy:full`, installing ROS deps and colcon-building `dtc-msgs`, `rtk-correction`, `MOCHA`, and `scoring-server-submission`.

## Run

```bash
./run-basestation.bash [--rtk] [--mocha] [--sender] [--all] [--gate N]
```

| Flag         | Effect                                                                              |
| ------------ | ----------------------------------------------------------------------------------- |
| `--rtk`    | Launch the RTK correction broadcaster                                               |
| `--mocha`  | Launch MOCHA (mesh networking)                                                      |
| `--sender` | Launch the scorecard submitter for the gate given by`--gate` (default gate `1`) |
| `--all`    | Enable RTK + MOCHA + sender together                                                |
| `--gate N` | Which gate to run when`--sender` is set: `1`–`4`, or `5` for HMT           |

If none of `--rtk`, `--mocha`, or `--sender` are passed, the container instead starts `basestation-supervisor.sh`, which auto-detects the RTK receiver (`/dev/ublox`) and Rajant radio (`rajant` interface) and starts/stops those services accordingly. Checks are run every 5 seconds.

> **Notes:**
>
> - Requires the scoring server to be reachable, or the scorecard-submitter launch will error.
> - `run-basestation.bash` mounts `./data` and `./scoring-server-submission` into the container, and expects a `.env` file next to the script (for RTK IP/port, Mapbox token, etc.)
