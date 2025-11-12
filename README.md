# picobot-firmware

Firmware for the Raspberry Pi Pico inside of my Stinger robot. Based on micro-ROS on FreeRTOS.

## Running teleop_twist_joy

```
# Run the joystick node
ros2 run joy joy_node -p deadzone:=0.166
```

```
# Run the /joy to /cmd_vel bridge
ros2 run teleop_twist_joy teleop_node --ros-args -p enable_button:=-1 -p require_enable_button:=false -p axis_linear.x:=4 -p axis_angular.yaw:=3 -p scale_linear.x:=1.0 -p scale_angular.yaw:=1.0
```
