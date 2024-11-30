#Template for projects

# File Structure

This repository contains (source code) that are intended to be executed from the docker container: `src stuff`.
A `Dockerfile`, along with bash scripts to build `dockerbuild.sh` and run `dockerrun.sh` the container are included.

### _**Local Machine**_
```
/current_directory/YOUR_REPO
├── data/
├── src/
│   ├── ...
```

When running the container using `dockerrun.sh`, the project directory is mounted as `/workdir`. This directory is treated as a ros2 workspace.
### _**Docker Container**_
```
/workdir
├── data/
├── src/
│   ├── ...
│   ├── ...
```

## Usage

Building and running the container

1 - Clone the repo and change directory into the repo:
`git@github.com:YOUR_REPO_HERE`
`cd YOUR_REPO`

2 - Build the container:
`sh dockerbuild.sh`

3 - Run the container:
`sh dockerrun.sh`

Add additional instructions as needed
