cmake_minimum_required(VERSION 3.0.2)

# EDIT: name of package
project(template_msg_pkg)

# EDIT: list all <build_depend> and <depend> package dependencies
find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
)

# EDIT: list all message files in the /msg folder
add_message_files(FILES
  template_msg.msg
)

# EDIT: list all packages containing msg dependencies
generate_messages(DEPENDENCIES
  std_msgs
)

# EDIT: list all <exec_depend> and <depend> packages
catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
)
