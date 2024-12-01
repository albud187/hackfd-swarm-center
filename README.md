
# Project Description

This repository contains two ROS2 packages in the  `src` directory: `drone_action_model`  `drone_ui`.

`drone_action_model` is the ROS2 package that implements motion planning and kinematics for a team of drones

`drone_ui` is the ROS2 package that implements a pygame graphical user interface (GUI) to facilitate user input and control of the team of drones.


# Requirements

This package was tested on ubuntu 22.04.3, and requires Docker.

A `Dockerfile`, along with bash scripts to build `dockerbuild.sh` and run `dockerrun.sh` the container are included. The `Dockerfile` is configured to install all dependancies.

This repository is intended to be executed from the included docker container. **It will not run otherwise, without modifications to file paths in the source code**

# File Overview

### _**Local Machine Direcory**_
```
/current_directory/swarm-ui/
|-- data/
|-- assets/
|-- src/
|   |-- drone_action_model/
|   |   |-- launch/
|   |   |-- src/
|-- |-- drone_ui/
|   |    |-- drone_ui/
```

When running the container using `dockerrun.sh`, the project directory (project root) is mounted as `/workdir`. This directory is treated as a ROS2 workspace.
### _**Docker Container Directory**_
```
/workdir/
|-- data/
|-- assets/
|-- src/
|   |-- drone_action_model/
|   |   |-- launch/
|   |   |-- src/
|-- |-- drone_ui/
|   |   |-- drone_ui/
```

The source code (C++) for `drone_action_model` is in `/workdir/src/drone_action_model/src/`

The source code (python) for `drone_ui` is in `/workdir/src/drone_ui/drone_ui/`

The files in `data/` and `assets/` contain essential information for running the `drone_action_model` and `drone_ui` ROS 2 packages.

## Setup

Building and running the container

1 - Clone the repo and change directory into the repo:

`git clone git@github.com:albud187/hackfd-swarm-center.git`

`cd hackfd-swarm-center`

2 - Build the container:
`bash dockerbuild.sh`

3 - Run the container:
`bash dockerrun.sh`

4 - Build ROS2 packages and source setup files. On the container you will be on `/workdir/` (container project root). From this directory execute:

`colcon build`

`bash entrypoint.sh`

`source install/setup.bash`

If you only want to build one package (not both), you can execute either:

`colcon build --packages-select drone_action_model`

or

`colcon build --packages-select drone_ui`


## Usage

This repo has two scenario provided.

Scenario 1 has 3 drones under your control, with 3 targets. This scenario is launched by executing:
Scenario 2 is similar, except with 6 drones and 5 targets.

Scenario 1 is launched by executing:

`ros2 launch drone_ui launch_s1.py`


![Scenario 1](https://github.com/albud187/hackfd-swarm-center/blob/main/repo_files/launch_s1.JPG)


This will launch the simulation, along with the pygame GUI to control the drones. Friendly drones under your control are blue, while enemy targets (also drones) are red.
Selected friendly drones turn light blue. Selected enemy drones turn dark red. Locked-in enemy drones turn black.

Users will be able to perform the following actions

- Selecting objects. User can draw a box to select objects. Selected objects will change colour.

- Go to goal. Commands the selected friendly drones to move to the target location

- Low Hover / High Hover. Commeands the selected friendly drones to hover at a low or high altitude.

- Add targets. Locks targets for attacking. Requires prior selection of enemy drones. Locked-in targets turn black.

- Attack. Commands friendly drone(s) to fly into the locked targets. In the case of multiple friendly / enemy drones, linear sum assignment will be used to determine which drones are the attackers / targets.

- Clear locked targets. Removes locked-in enemy drones. On the GUI, they will turn back to red from black.


