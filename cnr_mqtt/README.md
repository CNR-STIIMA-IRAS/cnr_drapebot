# cnr_mqtt
The cnr_mqtt package implements communication from MQTT to ROS FollowJointTrajectoryAction.
## configuration
The configuration file is described below.

```
action_name: "/moveit_pkg/follow_joint_trajectory"          # the FolloJointTrajectoryAction name (the one defined in Moveit!) 
mqtt_topic: "/robot_1/joint_trajectory"                     # the MQTT topic from which the trajectory is received
broker_address: "169.254.252.101"                           # address of the MQTT broker (host)
port: 1883                                                  # porto of the host
client_id: mqtt_to_ros_converter                                   # ID of the executable
rate: 10                                                    # update rate 

joint_names:                                                # list of joints contained in the trajectory (must be the same as ros controllers)
- joint_0
- joint_1
- joint_2
- joint_3
- joint_4
- joint_5
- joint_6
```

