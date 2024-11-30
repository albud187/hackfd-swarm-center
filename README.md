
# File Overview

This repository two ROS2 packages in the  `src` directory: `drone_action_model`  `drone_ui`.

`drone_action_model` is the ROS2 package that implements motion planning and kinematics for a team of drones

`drone_action_model` is the ROS2 package that implements a pygame interface to facilitate user input and control of the team of drones.

This repository is intended to be ran from the included docker container.

A `Dockerfile`, along with bash scripts to build `dockerbuild.sh` and run `dockerrun.sh` the container are included.

### _**Local Machine Direcory**_
```
/current_directory/swarm-ui/
|-- data/
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
|-- src/
|   |-- drone_action_model/
|   |   |-- launch/
|   |   |-- src/
|-- |-- drone_ui/
|   |   |-- drone_ui/
```

The source code (C++) for `drone_action_model` is in `/workdir/src/drone_action_model/src/`

The source code (python) for `drone_ui` is in `/workdir/src/drone_ui/drone_ui/`

## Setup

Building and running the container

1 - Clone the repo and change directory into the repo:
`git@github.com:albud187/swarm-ui.git`
`cd swarm-ui`

2 - Build the container:
`bash dockerbuild.sh`

3 - Run the container:
`bash dockerrun.sh`

4 - Build ROS2 packages. On the container you will be on `/workdir/` (container project root) execute:

`colcon build`

`source install/setup.bash`

If you only want to build one package (not both), you can execute either:

`colcon build --packages-select drone_action_model`

or


`colcon build --packages-select drone_ui`
