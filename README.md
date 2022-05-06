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

