# drapecell_robot_bringup
The package drapecell_robot_bringup contains the launcher to control the DrapeBot DrapeCell.

## Example
To lanch the DrapeCell controller:

1. Start 4 terminals.
2. **[Terminal 1]** Launch the example:
```
roslaunch drapecell_robot_bringup drapecell_6axis_robot.launch robot_ip:=<ROBOT IP ADDRESS>
```

3. **[Terminal 2]** To use RWS services:
```
rosservice call /rws/stop_rapid "{}"
rosservice call /rws/pp_to_main "{}"
rosservice call /rws/start_rapid "{}"
rosservice call /rws/sm_addin/start_egm_joint "{}"
```

4. **[Terminal 3]** To switch the controller:
```
rosservice call /egm/controller_manager/switch_controller {"start_controllers: [mqtt_to_joint_position_controller], stop_controllers: [''], strictness: 1, start_asap: false, timeout: 0.0"}
```

5. **[Terminal 4]** To send a control command through MQTT (optional only for testing the mqtt_to_joint_position_controller):
```
mosquitto_pub -h <BROKER_IP_ADDRESS> -t </robot_1/command> -m "7 double (6 robot positions + linear axis) + 1 unsigned long int (cycle counter)"
```

6. **[Terminal 3]** Use `rosservice` to stop the `EGM` session:
   ```
   rosservice call /rws/sm_addin/stop_egm "{}"
   ```