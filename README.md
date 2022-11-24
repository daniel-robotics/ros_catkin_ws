# ros_catkin_ws
A template catkin-tools workspace for ROS1-NOETIC, featuring built-in package management.

## Motivation:
ROS projects are incredibly non-portable by nature. A Catkin workspace consists of a collection of loosely stitched-together packages: downloaded code, system dependencies, configuration files, scripts, and custom modules. Attempting to migrate your codebase to a different system, or sharing it with others, can be a near-impossible feat due to slight variations between the different systems.  

This project attempts to create a managed Catkin workspace in which external dependencies and system configuration are better defined.

## Usage:
1. Install ROS using the **ros_noetic_scripts** package, or create your own **~/.rosconfig** file. See here: https://github.com/daniel-utilities/ros-noetic-scripts
2. Create a new repo based on this template, and clone it to your machine: `git clone --recursive URL`
3. Edit the file: **config/depends.conf** . Add here any external dependencies which are required by your ROS project. Supports 3 kinds of dependencies:  
     - APT_PACKAGES: packages to be installed through apt-get. Always prefer to install ROS packages this way if possible. Typically ROS packages in APT are listed as ros-noetic-PKG.
     - ROS_PACKAGES: ROS sources to be downloaded to the project's **src/** directory. Missing dependencies will be downloaded recursively. Any packages already installed to the global ROS installation will be skipped. See available packages at https://index.ros.org/packages/ .
     - GIT_REPOSITORIES: Git repos added here will be cloned to the project's **src/** directory. Frequently, Catkin packages are only available as git repos and are not found in APT or the ROS database.

4. Initialize the workspace: **scripts/init-workspace.sh** . This will initialize Catkin-Build, create the **src/** directory, and download the packages specified by **depends.conf**.  
5. Build the workspace: **scripts/build-workspace.sh** . 

If at any time additional external dependencies are needed, simply update **config/depends.conf** and run **scripts/install-depends.sh**.
