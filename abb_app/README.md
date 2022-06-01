# abb_app
The abb app package contains contains configuration files to load the control enviroment for the ABB robot.
## Index
Relevant are the following files:
1. under the config folder
    1. [drape_cell_hw.yaml](#dchw)
    2. [drape_cell_configurations.yaml](#dcc)
    3. [mqtt_config.yaml](#mqc)
2. under the launch folder:
    1. [cell_bringup.launch](#cbu)

## drapecell_hw.yaml <a name="dchw"></a>
This file contains the robot hardware description:
1. mqtt_hw: implements the [cnr_mqtt_hardware_interface](https://github.com/CNR-STIIMA-IRAS/cnr_mqtt_hardware_interface), refer to its page for documentation
2. deformation_hw: a TopicRobotHW that contains [deformation_ctrl](https://github.com/CNR-STIIMA-IRAS/deformation_ctrl)
3. plan_hw: a TopicRobotHW that contains a microinterpolator

## drapecell_configurations.yaml <a name="dcc"></a>
This file contains the possible control configurations:
1. use configuration "planner" to listen to a FollowJointTrajectoryAction, microinterpolates the received trajectory, passes it to deformation_ctrl and finally, via MQTT to the robot.

## mqtt_config.yaml <a name="mqc"></a>
This file contains information necessary for the cnr_mqtt node:
1. the [cnr_mqtt](https://github.com/CNR-STIIMA-IRAS/cnr_mqtt) node connects properly with MQTT and ROS actiFollowJointTrajectoryAction, refer to its page for documentation

## cell_bringup.launch <a name="cbu"></a>
This file load all the necessary to use the cell:

```
 <include file="$(find abb_moveit)/launch/planning_context.launch" >
    <arg name="load_robot_description" value="true" />
  </include>

 <include file="$(find abb_moveit)/launch/move_group.launch">
   <arg name="allow_trajectory_execution" value="true"/>
   <arg name="load_robot_description" value="false"/>
 </include>
 
  <!-- Run Rviz and load the default config to see the state of the move_group node -->
  <include file="$(find abb_app)/launch/moveit_rviz.launch" if="$(arg use_rviz)">
    <arg name="rviz_config" value="$(find abb_app)/config/moveit.rviz"/>
    <arg name="debug" value="$(arg debug)"/>
  </include>
```
loads moveit and rviz stuff.

```
 <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>

  <arg unless="$(arg debug)" name="launch_prefix" value="" />
  <arg     if="$(arg debug)" name="launch_prefix" value="gdb --ex run --args" />

  <node launch-prefix="$(arg launch_prefix)" pkg="cnr_configuration_manager" type="cnr_configuration_manager_node" name="configuration_manager" output="screen">
    <rosparam>
      appenders: [screen,file]
      levels: [info,trace]
      file_name: "configuration_manager"
      append_to_file: true
      pattern_layout: "[%5p][%d{HH:mm:ss,SSS}][%M:%04L][%c] %m%n"
    </rosparam>
    <rosparam command="load" file="$(find abb_app)/config/drape_cell_configurations.yaml" />
  </node>

  <node pkg="cnr_configuration_manager" type="dispatcher" name="configuration_dispatcher" output="screen" >
    <rosparam>
      appenders: [screen]
      levels: [info]
      file_name: "bbb"
      pattern_layout: "[%5p][%d{HH:mm:ss,SSS}][%M:%04L][%c] %m%n"
    </rosparam>
    <param name="num_worker_threads" value="40" />
    <remap from="~configuration_dispatches" to="/configuration_manager/configuration_dispatches" />
  </node>
```
loads cnr_ros_control environment

```
  <group ns="robot_1">
    <node pkg="cnr_mqtt" type="mqtt_converter" name="mqtt_traj_converter" output="screen"/>
    <rosparam command="load" file="$(find abb_app)/config/mqtt_config.yaml" />
  </group>
```
loads the cnr_mqtt node and required params.

