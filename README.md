# dt-duckiebot-localize

***

This repository is a combination of two ROS-based
repositories for Duckietown.

**NOTE:** This repo is developed from a boilerplate 
repository intended for ROS. Check out [the boilerplate](https://github.com/duckietown/template-ros).


## What it contains

### 1. dt-duckieOdometry

This repository contains a ROS package for publishing Odometry data from a dedicated node.
The repo has intended use against a DB21M model of duckiebots. [Link to repository](https://github.com/sondre5M1/dt-duckieOdometry)

### 2. dt_duckieImu

This repository contains a ROS package along with drivers for publishing IMU data from a dedicated node.
The repo is intended for a DB21M model as the package contains drivers for MPU9250. [Link to repository](https://github.com/reafed/dt_duckieImu)


## How to set it up

This is a simple tutorial for setting up and utilizing this package on your duckiebot.

**NOTE:** It requires that you have fully set up your duckiebot after the manual. 

### 1. Clone the repository

Clone this repository by using the following command:

> `git clone https://github.com/sondre5M1/dt-duckiebot-localize`

### 2. Build the packages to the desired duckiebot

Once the repository is finished with the cloning process
go to the path for the repository. Once there, run the command:

> `dts devel build -f -H <HOSTNAME>.local`

This may take some time to process as it instructs the duckiebot to create an image of the packages. 

### 3. Run the package on the desired duckiebot

As the build is complete all that requires now is to run the command:

> `dts devel run -H <HOSTNAME>.local -M -s -f -- --privileged -it -v /tmp/argus_socket:/tmp/argus_socket`

The command will instruct the duckiebot to run the image in a container.
This command must be executed each time the duckie is booted. 