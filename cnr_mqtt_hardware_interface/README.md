# cnr_mqtt_hardware_interface
The cnr_mqtt_hardware_interface implements communication to MQTT from cnr_ros_control framework.
## configuration
The configuration file is described below.

```
mqtt_hw:
  type: "cnr/control/MQTTRobotHW"
  appenders      : [file, screen]
  levels         : [trace, info]
  pattern_layout : "[%5p][%d{HH:mm:ss,SSS}][%50M:%04L][%24c] %m%n"
  file_name      : mqtt_hw
  sampling_period: 0.01
  diagnostic_period: 0.1
  maximum_missing_cycles: 100
  feedback_joint_state_timeout: 10

  joint_names:   
  - joint_0
  - joint_1
  - joint_2
  - joint_3
  - joint_4
  - joint_5
  - joint_6

  base_link: "base_link"                                            # base link of the kinematics chain
  tool_link: "tip_link"                                             # tool link of the kinematics chain
  robot_description_param: /robot_description
  robot_description_planning_param: /robot_description_planning
  kin_update_period: 0.1

  initial_position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]             
  use_delta_target_pos: false                                       # possible to send relative commands
  delta_pos_from_start: false                                       # relative commands wrt start of trajectory or previous cmd pos

  client_id: "MQTT_HW"                                              # MQTT host ID
  host: 169.254.57.188                                              # MQTT broker host address
  port: 1883                                                        # MQTT host port
  
  feedback_mqtt_topic: "robot1/feedback"                            # MQTT topic to subscribe to get feedback on the actual robot state
  out_feedback_mqtt_topic: "ITR_feedback"                           # MQTT topic to publish as feedback to previous actors
  command_mqtt_topic:  "robot1/command"                             # actual command, after trajectory deformation
  nominal_trajectory_topic: /joint_pos_target                       # nominal trajectory, from motion planner


  remap_source_args:
  - "/mqtt_hw/js_pub/joint_states"
  remap_target_args:
  - "/joint_states"
```

