cmake_minimum_required(VERSION 2.8.3)
project(marta_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs sensor_msgs message_generation)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(DIRECTORY msg)
 #    FILES
 #    AcousticModemStatus.msg
 #    Altitude.msg
 #    AttitudeEstimator.msg
 #    CANOpen.msg
 #    CommunicationMessage.msg
 #    FileTransfert.msg
 #    Countermeasure.msg
 #    CountermeasureList.msg
 #    Depth.msg
 #    Distance.msg
 #    DRError.msg
 #    Dvl.msg
 #    ErrorsTimed.msg
 #    Euler.msg
 #    FinderAnswer.msg
 #    FinderInfo.msg
 #    Finders.msg
 #    Forces.msg
 #    Gps.msg
 #    IDValue.msg
 #    Imu.msg
 #    Map.msg
 #    MapElement.msg
 #    Mission.msg
 #    Multibeam.msg
 #    NavStatus.msg
 #    NedCurrent.msg
 #    NEDFix.msg
 #    Object.msg
 #    ObjectDetection.msg
 #    ObjectDetectionArray.msg
 #    Perceptions.msg
 #    Pose.msg
 #    PoseTimed.msg
 #    Position.msg
 #    Pressure.msg
 #    Problem.msg
 #    P900.msg
 #    ReferenceGenerators.msg
 #    SensorSynchro.msg
 #    SideScanSonar.msg
 #    SLAM.msg
 #    SonarSpeed.msg
 #    Speed.msg
 #    Tau.msg
 #    TargetParams.msg
 #    TargetsPosition.msg
 #    Targets.msg
 #    Temperature.msg
 #    Thrust.msg
 #    ThrustHippo.msg
 #    Trackers.msg
 #    TransferProgress.msg
 #    USBLFix.msg
 #    USBLLLAFix.msg
 #    UTCTime.msg
 #    WallInfo.msg
 #    Walls.msg
 #    Waypoint.msg
 #    ObjectDetection.msg
 #    ObjectDetectionArray.msg
 #    Quaternion.msg
 # )

## Generate services in the 'srv' folder
 add_service_files(
    FILES
    Finder.srv
    Finderreset.srv
    CreateLawnMower.srv
 )

## Generate actions in the 'action' folder
#add_action_files(
#    DIRECTORY action
#   FILES
#    WaypointFinder.action
#newactionfile
#   Action1.action
#   Action2.action
#)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES #actionlib_msgs
   std_msgs  # Or other packages containing msgs
   geometry_msgs
   sensor_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES marta_msgs
#  CATKIN_DEPENDS actionlib_msgs
#  DEPENDS system_lib
#)
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
# add_library(marta_msgs
#   src/${PROJECT_NAME}/marta_msgs.cpp
# )

## Declare a cpp executable
# add_executable(marta_msgs_node src/marta_msgs_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(marta_msgs_node marta_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(marta_msgs_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS marta_msgs marta_msgs_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_marta_msgs.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
