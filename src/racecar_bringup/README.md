# RaceCar BringUp

This package contains the configuration and launch files to spin up the racecar stack.

## Controllers
To connect a controller to the hardware interface you can use the following CLI commands. For more info 
please refer to [ros2_control](https://github.com/ros-controls/ros2_control).

Show available interfaces:
`ros2 control list_hardware_interfaces`

This should output the available system interfaces:
```
axelbr@car3:/workspace# ros2 control list_hardware_interfaces
command interfaces
        engine/velocity [available] [unclaimed]
        servo/position [available] [unclaimed]
state interfaces
        engine/velocity
        servo/position
```

For now, we just configured two simple controllers that only forward commands to the hardware interface: `forward_velocity_controller` and `forward_steering_controller`.

They can be loaded and activated with the following command:
`ros2 control load_controller forward_steering_controller --set-state active`

Now the command interface should be claimed by the steering controller:
```
axelbr@car3:/workspace# ros2 control list_hardware_interfaces
command interfaces
        engine/velocity [available] [unclaimed]
        servo/position [available] [claimed]
state interfaces
        engine/velocity
        servo/position
```

The forward controllers just listen on a topic (e.g. `/forward_steering_controller/commands`) and forward the message to the coresponding hardware interface.