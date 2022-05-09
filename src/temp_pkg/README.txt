Catkin package file structure:

template_rospy_pkg      # Root folder defines name of Catkin package
|   launch                  # Launch folder holds .launch files, each of which starts one or more ROS node.
|   |
|   |   publisher.launch        # Starts the nodes/publisher node and passes along command-line arguments
|   |   subscriber.launch       # Starts the nodes/subscriber node and passes along command-line arguments
|   |
|   nodes                   # Nodes folder holds executable python files which require ROS
|   |
|   |   publisher               # Python script (no .py file extension) which starts a ROS node with a Publisher.
|   |   subscriber              # Python script (no .py file extension) which starts a ROS node with a Subscriber.
|   |
|   scripts                 # Scripts folder holds executable scripts for non-ROS functionality.
|   |
|   |   template_py_script      # Python script (no .py file extension) which does NOT start a ROS node (but still depends on "src")
|   |   template_sh_script      # Bash script (no .sh file extension)
|   |
|   src                     # Src folder holds all python source code
|   |
|   |   template_rospy_pkg      # Create a Python package with the same name as the Catkin package
|   |   |   
|   |   |   
|   |   |   
|   |   |   
|   |   CMakeLists.txt          # Catkin-required file which defines the build process  
|   |   package.xml             # Catkin-required file which defines the package (name, dependencies on other packages, etc)  
|   |   setup.py                # Installs nodes, scripts, launch files, and Python packages (src) such that other packages (and ROS) can find them