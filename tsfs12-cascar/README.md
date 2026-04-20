# Control Autonomous Systems CAR - Cascar

> Collective ROS code template for simulating, planning and controlling the RCCar platform @ Vehicular Systems to be used in
> the course in Learning, Planning, and Control of Autonomous vehicles (TSFS12).

**To get started, head over to the wiki pages where you will find detailed instructions on installation and how to start developing!**

## :calendar: Update history

### 2025

* Refactor launch files into single `demo` package.
* Move to [ROS2 Jazzy](https://docs.ros.org/en/jazzy/index.html) (instead of Humble)
* The cars are now preconfigured to automatically launch all required software on startup.This means you will not need to manually interact with the  Pi’s during normal operation.
* Include a [configuration file](config.toml) to centralize configuration.
* ROS2 namespaces are used to differentiate the cars (previously used `ROS2_DOMAIN_ID` environment varialbe for this).

### 2024

* Move to ROS2

### 2023

* Add Docker support
* Remove several templates
* Clean up .launch files
* Add support to steer simulation with keyboard
* Update wiki page

### 2022

* Update wiki page

### 2021

* Major rework of repository structure and functionality.
* Addition of the [simulator](src/simulator) package for offline simulation of various functionalities.
* Addition of various tools, packages, custom messages and basic car model for visualization.

### 2020

* Updated library to function with latest ROS (1) release: [Noetic](http://wiki.ros.org/noetic).
* Ported all Python2.7 code to Python3 standard.
* Addition of 6-DOF IMU and code to access readings. Current model is [MPU6050](https://www.elfa.se/en/mpu-6050-dof-accelerometer-and-gyroscopic-sensor-breakout-adafruit-3886/p/30167655?queryFromSuggest=true)
* Addition of packages for reading Qualisys and the [Rplidar](http://wiki.ros.org/rplidar) directly in the repo.

## :wrench: Installation

If this is your first time visiting, head over to the [installation](https://gitlab.liu.se/davax85/tsfs12-cascar/-/wikis/Installation) pages to setup your computer for development within the cascar framework.

## :books: Cascar basics

Once you have gone through the process of creating your cascar workspace, the [basics](https://gitlab.liu.se/davax85/tsfs12-cascar/-/wikis/Basics) pages contains easy access information on how to run code in the repository, send commands to the minicars as well as some inspiration packages for you to start you own development.

## :scroll: Package documentation

For detailed explanation of the provided content, head over to the [documentation](https://gitlab.liu.se/davax85/tsfs12-cascar/-/wikis/Documentation)  pages.

## :question: FAQ

On the [FAQ](https://gitlab.liu.se/davax85/tsfs12-cascar/-/wikis/FAQ)  pages we have collected some of the most common questions and corresponding answers that usually come up during the projects. If you have any questions or concerns, head over to the FAQs before consulting the supervisors.
